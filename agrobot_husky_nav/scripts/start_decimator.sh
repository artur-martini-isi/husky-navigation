#!/bin/bash
source /etc/clearpath/setup.bash
export ROS_SUPER_CLIENT=True
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/fastdds_big_msg.xml
exec python3 /home/robot/decimate.py "${1:-5}"
