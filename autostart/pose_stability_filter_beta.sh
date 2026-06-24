#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

if [ "${RC_POSE_FILTER_BETA:-0}" = "0" ]; then
  echo "[pose_filter_beta] RC_POSE_FILTER_BETA=0, disabled."
  exit 0
fi

INPUT_TOPIC="${RC_POSE_FILTER_BETA_INPUT_TOPIC:-${RC_POSE_FILTER_INPUT_TOPIC:-/robot/pose}}"
OUTPUT_TOPIC="${RC_BETA_FILTERED_POSE_TOPIC:-/robot/pose_filtered_beta}"
ALIAS_TOPIC="${RC_POSE_FILTER_BETA_ALIAS_TOPIC:-/lidar/pose_filtered_beta}"
STATUS_TOPIC="${RC_POSE_FILTER_BETA_STATUS_TOPIC:-/lidar/pose_filter_beta/status}"

echo "[pose_filter_beta] input=$INPUT_TOPIC"
echo "[pose_filter_beta] output=$OUTPUT_TOPIC"
echo "[pose_filter_beta] alias=$ALIAS_TOPIC"
echo "[pose_filter_beta] status=$STATUS_TOPIC"
echo "[pose_filter_beta] mode=beta"

exec ros2 run lidar_localization pose_stability_filter \
  --ros-args \
  -p input_pose_topic:="$INPUT_TOPIC" \
  -p output_pose_topic:="$OUTPUT_TOPIC" \
  -p output_pose_alias_topic:="$ALIAS_TOPIC" \
  -p status_topic:="$STATUS_TOPIC" \
  -p filter_mode:=beta \
  -p static_alpha:="${RC_POSE_FILTER_BETA_STATIC_ALPHA:-${RC_POSE_FILTER_STATIC_ALPHA:-0.12}}" \
  -p moving_alpha:="${RC_POSE_FILTER_BETA_MOVING_ALPHA:-${RC_POSE_FILTER_MOVING_ALPHA:-0.92}}" \
  -p static_speed_threshold:="${RC_POSE_FILTER_BETA_STATIC_SPEED:-${RC_POSE_FILTER_STATIC_SPEED:-0.03}}" \
  -p full_speed_threshold:="${RC_POSE_FILTER_BETA_FULL_SPEED:-${RC_POSE_FILTER_FULL_SPEED:-0.20}}" \
  -p static_yaw_rate_deg:="${RC_POSE_FILTER_BETA_STATIC_YAW_RATE_DEG:-${RC_POSE_FILTER_STATIC_YAW_RATE_DEG:-8.0}}" \
  -p full_yaw_rate_deg:="${RC_POSE_FILTER_BETA_FULL_YAW_RATE_DEG:-${RC_POSE_FILTER_FULL_YAW_RATE_DEG:-45.0}}" \
  -p spike_step_m:="${RC_POSE_FILTER_BETA_SPIKE_STEP:-0.012}" \
  -p beta_confirm_samples:="${RC_POSE_FILTER_BETA_CONFIRM_SAMPLES:-3}" \
  -p beta_return_tolerance_m:="${RC_POSE_FILTER_BETA_RETURN_TOLERANCE:-0.006}" \
  -p max_xy_jump:="${RC_POSE_FILTER_BETA_MAX_XY_JUMP:-${RC_POSE_FILTER_MAX_XY_JUMP:-0.50}}" \
  -p max_xy_speed:="${RC_POSE_FILTER_BETA_MAX_XY_SPEED:-${RC_POSE_FILTER_MAX_XY_SPEED:-5.0}}" \
  -p max_yaw_jump_deg:="${RC_POSE_FILTER_BETA_MAX_YAW_JUMP_DEG:-${RC_POSE_FILTER_MAX_YAW_JUMP_DEG:-90.0}}" \
  -p recovery_reject_count:="${RC_POSE_FILTER_BETA_RECOVERY_REJECT_COUNT:-${RC_POSE_FILTER_RECOVERY_REJECT_COUNT:-10}}" \
  -p recovery_time_sec:="${RC_POSE_FILTER_BETA_RECOVERY_TIME:-${RC_POSE_FILTER_RECOVERY_TIME:-1.0}}"
