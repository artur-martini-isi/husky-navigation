#!/usr/bin/env python3
"""Run ON THE ROBOT with the nav stack running. Builds a mission from waypoints given in the robot's
current body frame (forward, left) in metres, using the follower's current pose from its status topic.
Usage: make_relative_mission.py out.yaml "fwd,left" "fwd,left" ...   e.g. square: "5,0" "5,5" "0,5" "0,0" """
import json, math, sys, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String
R = 6371008.8
out = sys.argv[1]; pts = [tuple(float(v) for v in a.split(',')) for a in sys.argv[2:]]
rclpy.init(); n = Node('make_mission', namespace='a300_00096'); st = {}
n.create_subscription(String, 'gps_waypoint_follower/status', lambda m: st.update(json.loads(m.data)), 10)
t0 = time.time()
while time.time() - t0 < 30 and (st.get('lat') is None or st.get('yaw_deg') is None): rclpy.spin_once(n, timeout_sec=0.2)
if st.get('lat') is None or st.get('yaw_deg') is None: print('ERROR: no pose in status'); sys.exit(1)
lat, lon, yaw = st['lat'], st['lon'], math.radians(st['yaw_deg'])
lines = ['waypoints:', '  gps_points:']
print('origin: lat %.7f lon %.7f yaw %.1f deg ENU' % (lat, lon, st['yaw_deg']))
for fwd, left in pts:
    de = fwd * math.cos(yaw) - left * math.sin(yaw); dn = fwd * math.sin(yaw) + left * math.cos(yaw)
    la = lat + math.degrees(dn / R); lo = lon + math.degrees(de / (R * math.cos(math.radians(lat))))
    lines += ['    - latitude: %.8f' % la, '      longitude: %.8f' % lo, '      tolerance: 0.6']
    print('  fwd %+5.1f left %+5.1f -> lat %.7f lon %.7f' % (fwd, left, la, lo))
open(out, 'w').write('\n'.join(lines) + '\n'); print('wrote', out)
