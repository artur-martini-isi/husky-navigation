#!/usr/bin/env python3
"""Run ON THE ROBOT with the nav stack running. Writes a mission with one waypoint N metres ahead
of the robot, using the pose the gps_waypoint_follower itself is using (status topic, so it works
for both pose_source=fusion and pose_source=dual_gnss).
Usage: make_forward_mission.py <metres> [out.yaml]"""
import json, math, sys, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String
R = 6371008.8
dist = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
out = sys.argv[2] if len(sys.argv) > 2 else '/home/robot/mission_forward.yaml'
rclpy.init(); n = Node('make_mission', namespace='a300_00096'); st = {}
n.create_subscription(String, 'gps_waypoint_follower/status', lambda m: st.update(json.loads(m.data)), 10)
t0 = time.time()
while time.time() - t0 < 30 and st.get('lat') is None or st.get('yaw_deg') is None:
    rclpy.spin_once(n, timeout_sec=0.2)
    if time.time() - t0 > 30: break
if st.get('lat') is None or st.get('yaw_deg') is None:
    print('ERROR: follower has no pose yet. status: %s' % json.dumps(st)[:300]); sys.exit(1)
lat, lon, yaw = st['lat'], st['lon'], math.radians(st['yaw_deg'])
de, dn = dist * math.cos(yaw), dist * math.sin(yaw)
lat1 = lat + math.degrees(dn / R); lon1 = lon + math.degrees(de / (R * math.cos(math.radians(lat))))
open(out, 'w').write('waypoints:\n  gps_points:\n    - latitude: %.8f\n      longitude: %.8f\n      tolerance: 0.6\n' % (lat1, lon1))
print('robot: lat %.7f lon %.7f yaw %.1f deg ENU (source %s, health %s)' % (lat, lon, st['yaw_deg'], st.get('pose_source'), st.get('health')))
print('waypoint %.1f m ahead: lat %.7f lon %.7f -> %s' % (dist, lat1, lon1, out))
