#!/bin/bash
# Run ON THE ROBOT after the laptop started tools/ntrip_tunnel.py. Points ntrip_client at the tunnel.
source /etc/clearpath/setup.bash; export ROS_SUPER_CLIENT=True; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/fastdds_big_msg.xml
ss -tln | grep -q ":2101 " || { echo "tunnel port 2101 not listening on the robot"; exit 1; }
sed "s/host: .*/host: 127.0.0.1/" /home/robot/ntrip_ibge.yaml > /home/robot/ntrip_tunnel.yaml
/home/robot/nav_kill.sh ntrip >/dev/null
setsid nohup ros2 launch agrobot_husky_nav ntrip.launch.py params_file:=/home/robot/ntrip_tunnel.yaml > /tmp/ntrip.log 2>&1 < /dev/null &
sleep 12; grep -E "Connected|rror|xception" /tmp/ntrip.log | tail -2 | cut -c1-160
