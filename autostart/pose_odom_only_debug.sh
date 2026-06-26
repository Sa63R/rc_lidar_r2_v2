#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

OUTPUT_POSE_TOPIC="${RC_ODOM_ONLY_POSE_TOPIC:-/robot/pose_odom_only}"
OUTPUT_ALIAS_TOPIC="${RC_ODOM_ONLY_ALIAS_TOPIC:-/robot/pose_odom_only_alias}"
OUTPUT_ODOM_TOPIC="${RC_ODOM_ONLY_ODOM_TOPIC:-/robot/odom_odom_only}"
OUTPUT_POINT_TOPIC="${RC_ODOM_ONLY_POINT_TOPIC:-/robot/point_odom_only}"
STATUS_TOPIC="${RC_ODOM_ONLY_YAW_STATUS_TOPIC:-/robot/yaw_fusion_odom_only/status}"
ZERO_FRAME="${RC_POSE_ZERO_FRAME_ID:-robot_origin}"

echo "[pose_odom_only] input=/Odometry output_pose=$OUTPUT_POSE_TOPIC output_odom=$OUTPUT_ODOM_TOPIC"
echo "[pose_odom_only] output_alias=$OUTPUT_ALIAS_TOPIC reference_frame=$ZERO_FRAME yaw_fusion=0"

exec ros2 launch lidar_localization lidar_pose_from_odom.launch.py \
  node_name:=lidar_pose_from_odom_only_debug \
  input_odom_topic:=/Odometry \
  output_pose_topic:="$OUTPUT_POSE_TOPIC" \
  output_pose_alias_topic:="$OUTPUT_ALIAS_TOPIC" \
  output_point_topic:="$OUTPUT_POINT_TOPIC" \
  output_odom_topic:="$OUTPUT_ODOM_TOPIC" \
  zero_on_first_pose:=false \
  zero_frame_id:="$ZERO_FRAME" \
  reference_pose_enabled:=true \
  reference_frame_id:="$ZERO_FRAME" \
  output_stamp_mode:="${RC_ODOM_ONLY_STAMP_MODE:-now}" \
  yaw_fusion_enabled:=false \
  yaw_fusion_status_topic:="$STATUS_TOPIC"
