#!/usr/bin/env python3
"""GPS waypoint follower for the Husky A300 (Fixposition INS + LaserScan obstacle guard).

Design goals: no Nav2, no map. The Fixposition fused solution gives lat/lon and ENU yaw;
a P-controller steers towards the next waypoint; a LaserScan (Livox MID360 through
pointcloud_to_laserscan) slows down / stops the robot when something is in front.

Safety layers, from highest priority to lowest:
  1. Platform e-stop / safety stop (twist_mux locks) - hardware, outside this node.
  2. Joystick: holding the teleop enable button (L1) makes twist_mux prefer joy_teleop/cmd_vel
     (priority 10 > 1). This node also PAUSES while L1 is held so it does not fight when released.
     Circle (button 1) aborts the mission. Options (button 9) starts / resumes it.
  3. Joystick watchdog: if no /joy message arrives for `joy_timeout` s, the mission pauses.
  4. Fixposition health: mission pauses if the fused solution is stale or not globally initialised.
  5. Obstacle guard from the LaserScan.

Topics (relative to the robot namespace, e.g. /a300_00096):
  sub  sensors/ins_0/odom                          nav_msgs/Odometry   (FP_ENU0 frame, yaw source)
  sub  sensors/ins_0/fixposition/odometry_llh      sensor_msgs/NavSatFix (fused lat/lon)
  sub  sensors/ins_0/fixposition/fpa/odomstatus    fixposition_driver_msgs/FpaOdomstatus
  sub  scan                                         sensor_msgs/LaserScan
  sub  joy_teleop/joy                               sensor_msgs/Joy
  sub  gps_waypoint_follower/load_mission           std_msgs/String (path to a mission YAML)
  pub  cmd_vel                                      geometry_msgs/TwistStamped
  pub  gps_waypoint_follower/status                 std_msgs/String (JSON)
  srv  gps_waypoint_follower/{start,pause,stop}     std_srvs/Trigger
"""
import json
import math
import time

import rclpy
import yaml
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, LaserScan, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .geo import distance_bearing_enu, wrap_angle, yaw_from_quaternion

try:
    from fixposition_driver_msgs.msg import FpaOdomstatus, FpaGnsscorr
except ImportError:  # allows running without the driver messages (status check disabled)
    FpaOdomstatus = FpaGnsscorr = None

IDLE, RUNNING, PAUSED, BLOCKED, DONE, ABORTED = 'IDLE', 'RUNNING', 'PAUSED', 'BLOCKED', 'DONE', 'ABORTED'


class GpsWaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')

        # --- parameters -------------------------------------------------------------------
        p = self.declare_parameter
        p('mission_file', '')
        p('autostart', False)
        p('control_rate', 10.0)
        p('max_linear', 0.6)             # m/s
        p('min_linear', 0.15)            # m/s, keeps the robot creeping when aligned
        p('max_angular', 0.6)            # rad/s
        p('k_angular', 1.2)              # rad/s per rad of heading error
        p('turn_in_place_angle', 1.0)    # rad; above this, rotate without advancing
        p('slowdown_angle', 0.6)         # rad; linear speed scales down towards this error
        p('slowdown_distance', 2.0)      # m; linear speed scales down close to the waypoint
        p('waypoint_tolerance', 0.8)     # m
        p('waypoint_timeout', 180.0)     # s per waypoint before abort
        p('heading_offset_deg', 0.0)     # add if the Fixposition body x-axis is not the robot's forward
        p('require_global_init', True)   # require Fixposition init_status == GLOBAL_INIT (2)
        p('pose_timeout', 1.0)           # s; pause if fused pose is older than this
        # pose_source: 'fusion' uses the Fixposition fused ENU odom + LLH; 'dual_gnss' uses the two
        # raw antenna fixes (gps_0 = GNSS1, gps_1 = GNSS2): position = midpoint, heading = baseline.
        p('pose_source', 'fusion')
        p('dual_gnss_yaw_offset_deg', 90.0)  # yaw = bearing(GNSS1 -> GNSS2) + offset. 90 if GNSS1 is the LEFT antenna
        p('dual_gnss_min_fix', 7)        # FP_A-GNSSCORR fix type: 7 = RTK float, 8 = RTK fixed
        p('dual_gnss_yaw_alpha', 0.5)    # exponential smoothing of the baseline heading (1.0 = none)
        # obstacle guard
        p('obstacle_enabled', True)
        p('obstacle_half_angle', 0.6)    # rad; front sector = +-half_angle around x-axis
        p('obstacle_stop_distance', 1.2)  # m
        p('obstacle_slow_distance', 3.0)  # m
        p('obstacle_min_points', 3)      # ignore isolated returns
        p('obstacle_steer', False)       # when True, bias heading away from the fuller side
        p('obstacle_steer_gain', 0.4)    # rad/s
        p('scan_timeout', 1.0)           # s; stop if no scan arrives (only if obstacle_enabled)
        # joystick
        p('require_joy', True)
        p('joy_timeout', 1.0)            # s
        p('joy_start_button', 9)         # PS4 Options
        p('joy_stop_button', 1)          # PS4 Circle
        p('joy_teleop_enable_button', 4)  # PS4 L1 (teleop_twist_joy enable_button)
        p('joy_turbo_button', 5)         # PS4 R1

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.rate = float(g('control_rate'))
        self.max_lin, self.min_lin = float(g('max_linear')), float(g('min_linear'))
        self.max_ang, self.k_ang = float(g('max_angular')), float(g('k_angular'))
        self.turn_in_place = float(g('turn_in_place_angle'))
        self.slow_ang, self.slow_dist = float(g('slowdown_angle')), float(g('slowdown_distance'))
        self.tol, self.wp_timeout = float(g('waypoint_tolerance')), float(g('waypoint_timeout'))
        self.heading_offset = math.radians(float(g('heading_offset_deg')))
        self.require_global_init, self.pose_timeout = bool(g('require_global_init')), float(g('pose_timeout'))
        self.pose_source = str(g('pose_source'))
        self.dg_yaw_offset = math.radians(float(g('dual_gnss_yaw_offset_deg')))
        self.dg_min_fix, self.dg_alpha = int(g('dual_gnss_min_fix')), float(g('dual_gnss_yaw_alpha'))
        self.obs_on, self.obs_half = bool(g('obstacle_enabled')), float(g('obstacle_half_angle'))
        self.obs_stop, self.obs_slow = float(g('obstacle_stop_distance')), float(g('obstacle_slow_distance'))
        self.obs_min_pts, self.obs_steer = int(g('obstacle_min_points')), bool(g('obstacle_steer'))
        self.obs_steer_gain, self.scan_timeout = float(g('obstacle_steer_gain')), float(g('scan_timeout'))
        self.require_joy, self.joy_timeout = bool(g('require_joy')), float(g('joy_timeout'))
        self.btn_start, self.btn_stop = int(g('joy_start_button')), int(g('joy_stop_button'))
        self.btn_enable, self.btn_turbo = int(g('joy_teleop_enable_button')), int(g('joy_turbo_button'))

        # --- state ------------------------------------------------------------------------
        self.state = IDLE
        self.waypoints = []            # list of dicts: lat, lon, tolerance
        self.wp_index = 0
        self.wp_started_at = None
        self.lat = self.lon = None
        self.llh_time = 0.0
        self.yaw = None
        self.yaw_time = 0.0
        self.init_status = None
        self.status_time = 0.0
        self.ant = [None, None]        # (lat, lon, time) of GNSS1 / GNSS2 raw fixes
        self.gnss_fix = [None, None]   # FP_A-GNSSCORR fix types
        self.gnss_fix_time = 0.0
        self.front_min = math.inf
        self.left_free = self.right_free = math.inf
        self.scan_time = 0.0
        self.joy_time = 0.0
        self.joy_enable_held = False
        self.prev_buttons = []
        self.last_reason = ''

        # --- ROS I/O ----------------------------------------------------------------------
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)
        if self.pose_source == 'dual_gnss':
            self.create_subscription(NavSatFix, 'sensors/ins_0/gps_0/fix', lambda m: self.on_ant(0, m), sensor_qos)
            self.create_subscription(NavSatFix, 'sensors/ins_0/gps_1/fix', lambda m: self.on_ant(1, m), sensor_qos)
            if FpaGnsscorr is not None:
                self.create_subscription(FpaGnsscorr, 'sensors/ins_0/fixposition/fpa/gnsscorr',
                                         self.on_gnsscorr, sensor_qos)
        else:
            self.create_subscription(Odometry, 'sensors/ins_0/odom', self.on_odom, sensor_qos)
            self.create_subscription(NavSatFix, 'sensors/ins_0/fixposition/odometry_llh', self.on_llh, sensor_qos)
            if FpaOdomstatus is not None:
                self.create_subscription(FpaOdomstatus, 'sensors/ins_0/fixposition/fpa/odomstatus',
                                         self.on_odomstatus, sensor_qos)
        self.create_subscription(LaserScan, 'scan', self.on_scan, sensor_qos)
        self.create_subscription(Joy, 'joy_teleop/joy', self.on_joy, 10)
        self.create_subscription(String, '~/load_mission', self.on_load_mission, 10)
        self.create_subscription(String, '~/set_mission', self.on_set_mission, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_service(Trigger, '~/start', self.srv_start)
        self.create_service(Trigger, '~/pause', self.srv_pause)
        self.create_service(Trigger, '~/stop', self.srv_stop)
        self.create_timer(1.0 / self.rate, self.control_step)
        self.create_timer(0.5, self.publish_status)

        mission = str(g('mission_file'))
        if mission:
            self.load_mission(mission)
            if bool(g('autostart')) and self.waypoints:
                self.start('autostart')
        self.get_logger().info('gps_waypoint_follower ready (state=%s, %d waypoints)' %
                               (self.state, len(self.waypoints)))

    # ---------------------------------------------------------------- callbacks
    def now(self):
        return time.monotonic()

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 1.0 and \
                msg.pose.pose.position.x == 0.0 and msg.pose.pose.position.y == 0.0:
            return  # driver publishes zeros before the fusion is initialised
        self.yaw = wrap_angle(yaw_from_quaternion(q.x, q.y, q.z, q.w) + self.heading_offset)
        self.yaw_time = self.now()

    def on_llh(self, msg: NavSatFix):
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        self.lat, self.lon = msg.latitude, msg.longitude
        self.llh_time = self.now()

    def on_odomstatus(self, msg):
        self.init_status = int(msg.init_status)
        self.status_time = self.now()

    def on_gnsscorr(self, msg):
        self.gnss_fix = [int(msg.gnss1_fix), int(msg.gnss2_fix)]
        self.gnss_fix_time = self.now()

    def on_ant(self, idx: int, msg: NavSatFix):
        if msg.status.status < 0 or msg.latitude == 0.0:
            return
        self.ant[idx] = (msg.latitude, msg.longitude, self.now())
        a1, a2 = self.ant
        if a1 is None or a2 is None or abs(a1[2] - a2[2]) > 0.5:
            return
        # position: midpoint of the two antennas; heading: baseline GNSS1 -> GNSS2 plus mounting offset
        self.lat, self.lon = (a1[0] + a2[0]) / 2.0, (a1[1] + a2[1]) / 2.0
        self.llh_time = self.now()
        _, bearing = distance_bearing_enu(a1[0], a1[1], a2[0], a2[1])
        yaw = wrap_angle(bearing + self.dg_yaw_offset + self.heading_offset)
        if self.yaw is None or self.dg_alpha >= 1.0:
            self.yaw = yaw
        else:  # smooth on the circle
            self.yaw = math.atan2((1 - self.dg_alpha) * math.sin(self.yaw) + self.dg_alpha * math.sin(yaw),
                                  (1 - self.dg_alpha) * math.cos(self.yaw) + self.dg_alpha * math.cos(yaw))
        self.yaw_time = self.now()

    def on_scan(self, msg: LaserScan):
        front, left, right = [], [], []
        ang = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max and math.isfinite(r):
                a = wrap_angle(ang)
                if abs(a) <= self.obs_half:
                    front.append(r)
                elif 0 < a <= 2 * self.obs_half:
                    left.append(r)
                elif -2 * self.obs_half <= a < 0:
                    right.append(r)
            ang += msg.angle_increment
        front.sort()
        # robust minimum: ignore isolated returns
        self.front_min = front[self.obs_min_pts - 1] if len(front) >= self.obs_min_pts else math.inf
        self.left_free = sum(left) / len(left) if left else math.inf
        self.right_free = sum(right) / len(right) if right else math.inf
        self.scan_time = self.now()

    def on_joy(self, msg: Joy):
        self.joy_time = self.now()
        b = list(msg.buttons)
        pressed = lambda i: i < len(b) and b[i] == 1 and (i >= len(self.prev_buttons) or self.prev_buttons[i] == 0)  # noqa: E731
        self.joy_enable_held = (self.btn_enable < len(b) and b[self.btn_enable] == 1) or \
                               (self.btn_turbo < len(b) and b[self.btn_turbo] == 1)
        if pressed(self.btn_stop):
            self.stop('joystick stop button')
        elif pressed(self.btn_start):
            if self.state in (IDLE, PAUSED, BLOCKED, DONE, ABORTED):
                self.start('joystick start button')
        self.prev_buttons = b

    def on_load_mission(self, msg: String):
        self.load_mission(msg.data)

    def on_set_mission(self, msg: String):
        """Mission sent inline as JSON/YAML text (used by the web UI)."""
        try:
            data = yaml.safe_load(msg.data)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error('set_mission: cannot parse: %s' % e)
            return
        self.set_mission(data, 'set_mission topic')

    # ---------------------------------------------------------------- mission control
    def load_mission(self, path: str):
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error('failed to read mission %s: %s' % (path, e))
            return False
        return self.set_mission(data, path)

    def set_mission(self, data, source: str):
        """Accepts {'waypoints': {'gps_points': [...]}} / {'waypoints': [...]} / [...] with lat/lon items."""
        if self.state in (RUNNING, BLOCKED):
            self.stop('mission replaced while running')
        try:
            raw = data.get('waypoints', data) if isinstance(data, dict) else data
            if isinstance(raw, dict):
                raw = raw.get('gps_points', [])
            wps = []
            for w in raw:
                wps.append({'lat': float(w.get('latitude', w.get('lat'))),
                            'lon': float(w.get('longitude', w.get('lon'))),
                            'tolerance': float(w.get('tolerance', self.tol))})
            if not wps:
                raise ValueError('no waypoints found')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error('failed to load mission from %s: %s' % (source, e))
            return False
        self.waypoints, self.wp_index, self.wp_started_at = wps, 0, None
        self.state = IDLE
        self.last_reason = 'mission loaded'
        self.get_logger().info('loaded %d waypoints from %s' % (len(wps), source))
        return True

    def start(self, reason: str):
        if not self.waypoints:
            self.get_logger().warn('start requested but no mission loaded')
            return False
        if self.state in (DONE, ABORTED):
            self.wp_index = 0
        self.wp_started_at = self.now()
        self.state = RUNNING
        self.last_reason = reason
        self.get_logger().info('mission START (%s), waypoint %d/%d' %
                               (reason, self.wp_index + 1, len(self.waypoints)))
        return True

    def pause(self, reason: str):
        if self.state in (RUNNING, BLOCKED):
            self.state = PAUSED
            self.last_reason = reason
            self.get_logger().warn('mission PAUSED: %s' % reason)
            self.send_cmd(0.0, 0.0)

    def stop(self, reason: str):
        if self.state not in (IDLE, DONE):
            self.get_logger().warn('mission ABORTED: %s' % reason)
        self.state = ABORTED if self.state not in (IDLE, DONE) else self.state
        self.last_reason = reason
        self.send_cmd(0.0, 0.0)

    def srv_start(self, req, res):
        res.success = self.start('service')
        res.message = self.state
        return res

    def srv_pause(self, req, res):
        self.pause('service')
        res.success, res.message = True, self.state
        return res

    def srv_stop(self, req, res):
        self.stop('service')
        res.success, res.message = True, self.state
        return res

    # ---------------------------------------------------------------- control loop
    def health_problem(self):
        """Return a reason string when the mission must not drive, else None."""
        t = self.now()
        if self.require_joy and t - self.joy_time > self.joy_timeout:
            return 'joystick not connected'
        if self.joy_enable_held:
            return 'joystick teleop active'
        if self.lat is None or self.yaw is None:
            return 'waiting for Fixposition fused pose'
        if t - self.llh_time > self.pose_timeout or t - self.yaw_time > self.pose_timeout:
            return 'Fixposition pose stale'
        if self.pose_source == 'dual_gnss':
            if FpaGnsscorr is not None:
                if self.gnss_fix[0] is None or t - self.gnss_fix_time > 3.0:
                    return 'GNSS correction status unknown'
                if min(self.gnss_fix) < self.dg_min_fix:
                    return 'GNSS fix too weak for baseline heading (gnss1=%d gnss2=%d, need >=%d)' % (
                        self.gnss_fix[0], self.gnss_fix[1], self.dg_min_fix)
        elif self.require_global_init and FpaOdomstatus is not None:
            if self.init_status is None or t - self.status_time > 2.0:
                return 'Fixposition status unknown'
            if self.init_status < 2:
                return 'Fixposition fusion not globally initialised (init_status=%d)' % self.init_status
        if self.obs_on and t - self.scan_time > self.scan_timeout:
            return 'no LaserScan'
        return None

    def control_step(self):
        if self.state not in (RUNNING, BLOCKED):
            return
        problem = self.health_problem()
        if problem:
            # joystick override is a pause; sensor problems are pauses too (resume with Options)
            self.pause(problem)
            return

        wp = self.waypoints[self.wp_index]
        dist, bearing = distance_bearing_enu(self.lat, self.lon, wp['lat'], wp['lon'])
        if dist <= wp['tolerance']:
            self.get_logger().info('waypoint %d/%d reached (%.2f m)' %
                                   (self.wp_index + 1, len(self.waypoints), dist))
            self.wp_index += 1
            self.wp_started_at = self.now()
            if self.wp_index >= len(self.waypoints):
                self.state = DONE
                self.last_reason = 'mission complete'
                self.send_cmd(0.0, 0.0)
                self.get_logger().info('mission DONE')
            return
        if self.now() - self.wp_started_at > self.wp_timeout:
            self.stop('waypoint %d timeout' % (self.wp_index + 1))
            return

        err = wrap_angle(bearing - self.yaw)
        ang = max(-self.max_ang, min(self.max_ang, self.k_ang * err))
        if abs(err) > self.turn_in_place:
            lin = 0.0
        else:
            align = max(0.0, 1.0 - abs(err) / self.slow_ang)
            near = min(1.0, dist / self.slow_dist)
            lin = max(self.min_lin, self.max_lin * align * near)

        # obstacle guard
        if self.obs_on:
            if self.front_min <= self.obs_stop:
                self.state = BLOCKED
                self.last_reason = 'obstacle at %.2f m' % self.front_min
                self.send_cmd(0.0, 0.0)
                return
            if self.front_min < self.obs_slow:
                lin *= (self.front_min - self.obs_stop) / (self.obs_slow - self.obs_stop)
                if self.obs_steer:
                    ang += self.obs_steer_gain if self.left_free >= self.right_free else -self.obs_steer_gain
                    ang = max(-self.max_ang, min(self.max_ang, ang))
            if self.state == BLOCKED:
                self.get_logger().info('path clear, resuming')
        self.state = RUNNING
        self.send_cmd(lin, ang)

    def send_cmd(self, lin: float, ang: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(lin)
        msg.twist.angular.z = float(ang)
        self.cmd_pub.publish(msg)

    def publish_status(self):
        wp = self.waypoints[self.wp_index] if self.wp_index < len(self.waypoints) else None
        dist = bearing = None
        if wp and self.lat is not None:
            dist, bearing = distance_bearing_enu(self.lat, self.lon, wp['lat'], wp['lon'])
        s = {
            'state': self.state, 'reason': self.last_reason,
            'waypoint': self.wp_index + 1 if wp else None, 'total': len(self.waypoints),
            'distance_m': round(dist, 2) if dist is not None else None,
            'heading_error_deg': round(math.degrees(wrap_angle(bearing - self.yaw)), 1)
            if bearing is not None and self.yaw is not None else None,
            'lat': self.lat, 'lon': self.lon,
            'yaw_deg': round(math.degrees(self.yaw), 1) if self.yaw is not None else None,
            'pose_source': self.pose_source,
            'fp_init_status': self.init_status,
            'gnss_fix': self.gnss_fix,
            'front_min_m': round(self.front_min, 2) if math.isfinite(self.front_min) else None,
            'joy_ok': self.now() - self.joy_time <= self.joy_timeout,
            'health': self.health_problem(),
            'wp_index': self.wp_index,
            'waypoints': [[round(w['lat'], 8), round(w['lon'], 8)] for w in self.waypoints],
            'max_linear': self.max_lin,
        }
        self.status_pub.publish(String(data=json.dumps(s)))


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
