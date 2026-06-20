#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

ZERO_DELAY="${RC_POSE_ZERO_DELAY_SEC:-5.0}"
ZERO_FRAME="${RC_POSE_ZERO_FRAME_ID:-init}"

echo "[pose_interface] mapping mode: input=/Odometry zero_on_first_pose=true zero_delay=$ZERO_DELAY zero_frame=$ZERO_FRAME"
exec ros2 launch lidar_localization lidar_pose_from_odom.launch.py \
  input_odom_topic:=/Odometry \
  zero_on_first_pose:=true \
  zero_delay_sec:="$ZERO_DELAY" \
  zero_frame_id:="$ZERO_FRAME" \
  reference_pose_enabled:=false
