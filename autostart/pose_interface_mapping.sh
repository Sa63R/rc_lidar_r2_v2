#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

TARGET_FRAME="${RC_ROBOT_POSE_TARGET_FRAME:-robot_origin}"
SOURCE_FRAME="${RC_ROBOT_POSE_SOURCE_FRAME:-robot_base_highrate}"
Z_SOURCE_FRAME="${RC_ROBOT_POSE_Z_SOURCE_FRAME:-robot_base}"

echo "[pose_interface] mapping mode: TF $TARGET_FRAME -> $SOURCE_FRAME publishes /robot/pose"
exec ros2 run lidar_localization pose_from_tf --ros-args \
  -p target_frame:="$TARGET_FRAME" \
  -p source_frame:="$SOURCE_FRAME" \
  -p z_source_frame:="$Z_SOURCE_FRAME" \
  -p publish_rate:="${RC_ROBOT_POSE_HZ:-50.0}" \
  -p max_tf_age_sec:="${RC_ROBOT_POSE_MAX_TF_AGE:-0.0}"
