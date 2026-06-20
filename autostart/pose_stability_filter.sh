#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

if [ "${RC_POSE_FILTER:-1}" = "0" ]; then
  echo "[pose_filter] RC_POSE_FILTER=0, disabled."
  exit 0
fi

INPUT_TOPIC="${RC_POSE_FILTER_INPUT_TOPIC:-/lidar/pose}"
OUTPUT_TOPIC="${RC_POSE_FILTER_OUTPUT_TOPIC:-/lidar/pose_filtered}"
ALIAS_TOPIC="${RC_POSE_FILTER_ALIAS_TOPIC:-/robot/pose_filtered}"
STATUS_TOPIC="${RC_POSE_FILTER_STATUS_TOPIC:-/lidar/pose_filter/status}"

echo "[pose_filter] input=$INPUT_TOPIC"
echo "[pose_filter] output=$OUTPUT_TOPIC"
echo "[pose_filter] alias=$ALIAS_TOPIC"
echo "[pose_filter] status=$STATUS_TOPIC"

exec ros2 run lidar_localization pose_stability_filter \
  --ros-args \
  -p input_pose_topic:="$INPUT_TOPIC" \
  -p output_pose_topic:="$OUTPUT_TOPIC" \
  -p output_pose_alias_topic:="$ALIAS_TOPIC" \
  -p status_topic:="$STATUS_TOPIC" \
  -p static_alpha:="${RC_POSE_FILTER_STATIC_ALPHA:-0.20}" \
  -p moving_alpha:="${RC_POSE_FILTER_MOVING_ALPHA:-0.65}" \
  -p static_speed_threshold:="${RC_POSE_FILTER_STATIC_SPEED:-0.05}" \
  -p max_xy_jump:="${RC_POSE_FILTER_MAX_XY_JUMP:-0.15}" \
  -p max_xy_speed:="${RC_POSE_FILTER_MAX_XY_SPEED:-2.0}" \
  -p max_yaw_jump_deg:="${RC_POSE_FILTER_MAX_YAW_JUMP_DEG:-20.0}" \
  -p recovery_reject_count:="${RC_POSE_FILTER_RECOVERY_REJECT_COUNT:-10}" \
  -p recovery_time_sec:="${RC_POSE_FILTER_RECOVERY_TIME:-1.0}"
