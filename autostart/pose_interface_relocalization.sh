#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

ZERO_FRAME="${RC_POSE_ZERO_FRAME_ID:-init}"

echo "[pose_interface] relocalization mode: input=/map_lidar/odom zero_on_first_pose=false reference_frame=$ZERO_FRAME"
exec ros2 launch lidar_localization lidar_pose_from_odom.launch.py \
  input_odom_topic:=/map_lidar/odom \
  zero_on_first_pose:=false \
  zero_frame_id:="$ZERO_FRAME" \
  reference_pose_enabled:=true \
  reference_frame_id:="$ZERO_FRAME"
