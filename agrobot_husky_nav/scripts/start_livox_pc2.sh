#!/bin/bash
# Livox Mid-360 -> PointCloud2, dentro do FastDDS Discovery Server do Clearpath
source /etc/clearpath/setup.bash
source /home/robot/livox-lidar/ws_livox/install/setup.bash
export ROS_SUPER_CLIENT=True
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/fastdds_big_msg.xml

CFG=/home/robot/livox-lidar/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json

exec ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args -r __node:=livox_lidar_publisher \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p lvx_file_path:=/home/livox/livox_test.lvx \
  -p user_config_path:=$CFG \
  -p cmdline_input_bd_code:=livox0000000001
