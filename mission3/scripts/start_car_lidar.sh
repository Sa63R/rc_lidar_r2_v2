#!/usr/bin/env bash
# Standalone driver for the car connection detected on 2026-10-22.
set -euo pipefail
TASK3_CAR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TASK3_CAR_CONFIG="$TASK3_CAR_ROOT/config/car_mid360_131.json"
set +u
source "${TASK1_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
source "${TASK1_BASE_SETUP:-/opt/ros/jazzy/setup.bash}"
set -u
printf '车载雷达：192.168.1.131；主机：192.168.1.5；配置：%s\n' "$TASK3_CAR_CONFIG"
exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_lidar_publisher \
  -p "user_config_path:=$TASK3_CAR_CONFIG" \
  -p xfer_format:=1 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p 'lvx_file_path:=""' \
  -p cmdline_input_bd_code:=livox0000000001
