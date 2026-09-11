#!/bin/bash
# Run ON THE ROBOT. Stops navigation-related processes safely. Usage: nav_kill.sh [nav|ntrip|livox|bridge|indoor|follow|explore|goto|slam|zed|all]
# Invoke by path; do not paste these patterns into an interactive shell command (pkill -f would match it).
what=${1:-nav}
kill_pat() { for p in $(pgrep -f "$1"); do [ "$p" != "$$" ] && [ "$p" != "$PPID" ] && kill "$p" 2>/dev/null; done; }
case "$what" in
  nav|all)   kill_pat "agrobot_husky_nav nav.launch"; kill_pat "agrobot_husky_nav bringup.launch"; kill_pat "gps_waypoint_follower"; kill_pat "pointcloud_to_laserscan_node"; kill_pat "livox_static_tf" ;;&
  ntrip|all) kill_pat "agrobot_husky_nav ntrip.launch"; kill_pat "ntrip_ros.py" ;;&
  bridge|all) kill_pat "agrobot_husky_nav agrobot_bridge" ;;&
  indoor|all) kill_pat "indoor.launch.py"; kill_pat "lib/agrobot_husky_nav/voxel_local_map" ;;&
  follow|all) kill_pat "agrobot_husky_nav follow_me.launch"; kill_pat "lib/agrobot_husky_nav/follow_me" ;;&
  explore|all) kill_pat "explore.launch.py"; kill_pat "lib/agrobot_husky_nav/explore" ;;&
  goto|all) kill_pat "goto_point.launch.py"; kill_pat "lib/agrobot_husky_nav/goto_point" ;;&
  slam|all)   kill_pat "agrobot_husky_nav slam.launch"; kill_pat "async_slam_toolbox_node" ;;&
  zed|all)    kill_pat "zed.launch.py"; kill_pat "lib/agrobot_husky_nav/zed_stereo" ;;&
  livox|all) kill_pat "livox_ros_driver2_node"; sleep 2; for p in $(pgrep -f "livox_ros_driver2_node"); do [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null; done ;;
esac
sleep 1
echo "nav: $(pgrep -cf 'gps_waypoint_follower')  ntrip: $(pgrep -cf 'ntrip_ros.py')  livox: $(pgrep -cf 'livox_ros_driver2_node')  explore: $(pgrep -cf 'lib/agrobot_husky_nav/explore')  goto: $(pgrep -cf 'lib/agrobot_husky_nav/goto_point')"
