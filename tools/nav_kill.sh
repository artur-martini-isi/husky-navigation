#!/bin/bash
# Run ON THE ROBOT. Stops navigation-related processes safely. Usage: nav_kill.sh [nav|ntrip|livox|all]
# Invoke by path; do not paste these patterns into an interactive shell command (pkill -f would match it).
what=${1:-nav}
kill_pat() { for p in $(pgrep -f "$1"); do [ "$p" != "$$" ] && [ "$p" != "$PPID" ] && kill "$p" 2>/dev/null; done; }
case "$what" in
  nav|all)   kill_pat "agrobot_husky_nav nav.launch"; kill_pat "agrobot_husky_nav bringup.launch"; kill_pat "gps_waypoint_follower"; kill_pat "pointcloud_to_laserscan_node"; kill_pat "livox_static_tf" ;;&
  ntrip|all) kill_pat "agrobot_husky_nav ntrip.launch"; kill_pat "ntrip_ros.py" ;;&
  livox|all) kill_pat "livox_ros_driver2_node"; sleep 2; for p in $(pgrep -f "livox_ros_driver2_node"); do [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null; done ;;
esac
sleep 1
echo "nav: $(pgrep -f 'gps_waypoint_follower' | grep -vc $$)  ntrip: $(pgrep -f 'ntrip_ros.py' | grep -vc $$)  livox: $(pgrep -f 'livox_ros_driver2_node' | grep -vc $$)"
