#!/bin/bash
# Run ON THE ROBOT. Brings up everything the waypoint test needs and prints a status line.
source /etc/clearpath/setup.bash; export ROS_SUPER_CLIENT=True; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/fastdds_big_msg.xml
FP=${FP_IP:-192.168.131.35}
if ping -c1 -W1 $FP >/dev/null 2>&1; then
  if ! journalctl -u clearpath-sensors --no-pager -n 200 | grep -q "Connected to tcpcli://$FP"; then
    echo "fixposition reachable but driver down -> restarting clearpath-sensors"; sudo systemctl restart clearpath-sensors; sleep 15
  fi
else echo "WARNING: Fixposition $FP not reachable"; fi
pgrep -f "livox_ros_driver2_nod[e]" >/dev/null || { echo "starting livox"; setsid nohup /home/robot/start_livox_pc2.sh > /tmp/livox_pc2.log 2>&1 < /dev/null & sleep 6; }
pgrep -f "ntrip_ro[s]" >/dev/null || { echo "starting ntrip"; setsid nohup ros2 launch agrobot_husky_nav ntrip.launch.py > /tmp/ntrip.log 2>&1 < /dev/null & sleep 6; }
if [ "${BRIDGE:-1}" = "1" ]; then
  pgrep -f "agrobot_bridge" >/dev/null || {
    echo "starting agrobot bridge (telemetria para o sistema Agrobot)";
    CFG=$(ros2 pkg prefix agrobot_husky_nav)/share/agrobot_husky_nav/config/agrobot_bridge.yaml;
    PYTHONPATH=/home/robot/pylibs:$PYTHONPATH setsid nohup ros2 run agrobot_husky_nav agrobot_bridge \
      --ros-args --params-file "$CFG" -r __ns:=/a300_00096 > /tmp/bridge.log 2>&1 < /dev/null &
    sleep 4; }
fi
pgrep -f "http.server 8088" >/dev/null || { echo "starting web UI on :8088"; setsid nohup python3 -m http.server 8088 --directory /home/robot/webui > /tmp/webui.log 2>&1 < /dev/null & }
echo "livox: $(pgrep -f 'livox_ros_driver2_nod[e]' | wc -l) ntrip: $(pgrep -f 'ntrip_ro[s]' | wc -l) bridge: $(pgrep -cf 'agrobot_bridg[e]') sensors: $(systemctl is-active clearpath-sensors) webui: http://$(hostname -I | awk '{print $1}'):8088"
