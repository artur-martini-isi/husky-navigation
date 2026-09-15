#!/usr/bin/env python3
"""Run ON THE ROBOT with the nav stack running. Validates the heading source against real motion.
  heading_check.py start   -> saves current pose
  heading_check.py end     -> after driving straight forward a few metres: compares the bearing of
                              the displacement with the heading reported before/after."""
import json, math, sys, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String
R = 6371008.8; F = '/tmp/heading_check_start.json'
rclpy.init(); n = Node('heading_check', namespace='a300_00096'); st = {}
n.create_subscription(String, 'gps_waypoint_follower/status', lambda m: st.update(json.loads(m.data)), 10)
t0 = time.time()
while time.time() - t0 < 30 and st.get('yaw_deg') is None: rclpy.spin_once(n, timeout_sec=0.2)
if st.get('yaw_deg') is None: print('no pose in status:', json.dumps(st)[:200]); sys.exit(1)
if sys.argv[1] == 'start':
    json.dump(st, open(F, 'w')); print('start: lat %.7f lon %.7f yaw %.1f gnss_fix %s' % (st['lat'], st['lon'], st['yaw_deg'], st.get('gnss_fix')))
else:
    s = json.load(open(F)); lat0 = math.radians(s['lat'])
    de = math.radians(st['lon'] - s['lon']) * math.cos(lat0) * R; dn = math.radians(st['lat'] - s['lat']) * R
    d = math.hypot(de, dn); brg = math.degrees(math.atan2(dn, de))
    def wrap(a): return (a + 180) % 360 - 180
    print('moved %.2f m, bearing of motion %.1f deg ENU' % (d, brg))
    print('heading reported: start %.1f, end %.1f deg' % (s['yaw_deg'], st['yaw_deg']))
    print('error (motion - heading_end): %+.1f deg   (motion - heading_start): %+.1f deg' % (wrap(brg - st['yaw_deg']), wrap(brg - s['yaw_deg'])))
    if d < 1.0: print('WARNING: moved less than 1 m, result unreliable')
