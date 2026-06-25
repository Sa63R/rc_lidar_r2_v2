#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

if [ "${RC_POSE_FILTER:-1}" = "0" ]; then
  echo "[pose_filter] RC_POSE_FILTER=0, disabled."
  exit 0
fi

INPUT_TOPIC="${RC_POSE_FILTER_INPUT_TOPIC:-/robot/pose}"
OUTPUT_TOPIC="${RC_POSE_FILTER_OUTPUT_TOPIC:-/robot/pose_filtered}"
ALIAS_TOPIC="${RC_POSE_FILTER_ALIAS_TOPIC:-/lidar/pose_filtered}"
STATUS_TOPIC="${RC_POSE_FILTER_STATUS_TOPIC:-/lidar/pose_filter/status}"
QUALITY_GATE="$(rc_bool "${RC_POSE_FILTER_QUALITY_GATE:-1}")"

echo "[pose_filter] input=$INPUT_TOPIC"
echo "[pose_filter] output=$OUTPUT_TOPIC"
echo "[pose_filter] alias=$ALIAS_TOPIC"
echo "[pose_filter] status=$STATUS_TOPIC"
echo "[pose_filter] mode=normal"
echo "[pose_filter] quality_gate=$QUALITY_GATE"
echo "[pose_filter] yaw_alpha=${RC_POSE_FILTER_YAW_ALPHA:-0.85}"

exec ros2 run lidar_localization pose_stability_filter \
  --ros-args \
  -p input_pose_topic:="$INPUT_TOPIC" \
  -p output_pose_topic:="$OUTPUT_TOPIC" \
  -p output_pose_alias_topic:="$ALIAS_TOPIC" \
  -p status_topic:="$STATUS_TOPIC" \
  -p filter_mode:=normal \
  -p output_stamp_mode:="${RC_POSE_FILTER_STAMP_MODE:-now}" \
  -p quality_gate_enabled:="$QUALITY_GATE" \
  -p quality_topic:="${RC_POSE_FILTER_QUALITY_TOPIC:-/map_lidar/quality}" \
  -p quality_timeout_sec:="${RC_POSE_FILTER_QUALITY_TIMEOUT:-1.0}" \
  -p quality_max_jump_m:="${RC_POSE_FILTER_QUALITY_MAX_JUMP:-0.08}" \
  -p quality_max_fitness:="${RC_POSE_FILTER_QUALITY_MAX_FITNESS:-0.20}" \
  -p static_alpha:="${RC_POSE_FILTER_STATIC_ALPHA:-0.12}" \
  -p moving_alpha:="${RC_POSE_FILTER_MOVING_ALPHA:-0.92}" \
  -p yaw_alpha:="${RC_POSE_FILTER_YAW_ALPHA:-0.85}" \
  -p static_speed_threshold:="${RC_POSE_FILTER_STATIC_SPEED:-0.03}" \
  -p full_speed_threshold:="${RC_POSE_FILTER_FULL_SPEED:-0.20}" \
  -p static_yaw_rate_deg:="${RC_POSE_FILTER_STATIC_YAW_RATE_DEG:-8.0}" \
  -p full_yaw_rate_deg:="${RC_POSE_FILTER_FULL_YAW_RATE_DEG:-45.0}" \
  -p spike_step_m:="${RC_POSE_FILTER_SPIKE_STEP:-0.05}" \
  -p max_xy_jump:="${RC_POSE_FILTER_MAX_XY_JUMP:-0.50}" \
  -p max_xy_speed:="${RC_POSE_FILTER_MAX_XY_SPEED:-5.0}" \
  -p max_yaw_jump_deg:="${RC_POSE_FILTER_MAX_YAW_JUMP_DEG:-90.0}" \
  -p recovery_reject_count:="${RC_POSE_FILTER_RECOVERY_REJECT_COUNT:-10}" \
  -p recovery_time_sec:="${RC_POSE_FILTER_RECOVERY_TIME:-1.0}"
