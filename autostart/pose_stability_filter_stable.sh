#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

case "${RC_POSE_FILTER_STABLE:-0}" in
  1|true|TRUE|yes|YES|on|ON)
    ;;
  *)
    echo "[pose_filter_stable] RC_POSE_FILTER_STABLE=0, disabled."
    exit 0
    ;;
esac

INPUT_TOPIC="${RC_POSE_FILTER_STABLE_INPUT_TOPIC:-${RC_POSE_FILTER_INPUT_TOPIC:-/robot/pose}}"
OUTPUT_TOPIC="${RC_STABLE_FILTERED_POSE_TOPIC:-/robot/pose_filtered_stable}"
ALIAS_TOPIC="${RC_POSE_FILTER_STABLE_ALIAS_TOPIC:-/lidar/pose_filtered_stable}"
STATUS_TOPIC="${RC_POSE_FILTER_STABLE_STATUS_TOPIC:-/lidar/pose_filter_stable/status}"
QUALITY_GATE="$(rc_bool "${RC_POSE_FILTER_STABLE_QUALITY_GATE:-${RC_POSE_FILTER_QUALITY_GATE:-1}}")"

echo "[pose_filter_stable] input=$INPUT_TOPIC"
echo "[pose_filter_stable] output=$OUTPUT_TOPIC"
echo "[pose_filter_stable] alias=$ALIAS_TOPIC"
echo "[pose_filter_stable] status=$STATUS_TOPIC"
echo "[pose_filter_stable] mode=stable"
echo "[pose_filter_stable] quality_gate=$QUALITY_GATE"
echo "[pose_filter_stable] yaw_alpha=${RC_POSE_FILTER_STABLE_YAW_ALPHA:-${RC_POSE_FILTER_YAW_ALPHA:-0.85}}"

exec ros2 run lidar_localization pose_stability_filter \
  --ros-args \
  -p input_pose_topic:="$INPUT_TOPIC" \
  -p output_pose_topic:="$OUTPUT_TOPIC" \
  -p output_pose_alias_topic:="$ALIAS_TOPIC" \
  -p status_topic:="$STATUS_TOPIC" \
  -p filter_mode:=stable \
  -p output_stamp_mode:="${RC_POSE_FILTER_STABLE_STAMP_MODE:-${RC_POSE_FILTER_STAMP_MODE:-now}}" \
  -p quality_gate_enabled:="$QUALITY_GATE" \
  -p quality_topic:="${RC_POSE_FILTER_QUALITY_TOPIC:-/map_lidar/quality}" \
  -p quality_timeout_sec:="${RC_POSE_FILTER_QUALITY_TIMEOUT:-1.0}" \
  -p quality_max_jump_m:="${RC_POSE_FILTER_QUALITY_MAX_JUMP:-0.08}" \
  -p quality_max_fitness:="${RC_POSE_FILTER_QUALITY_MAX_FITNESS:-0.20}" \
  -p static_alpha:="${RC_POSE_FILTER_STABLE_STATIC_ALPHA:-${RC_POSE_FILTER_STATIC_ALPHA:-0.12}}" \
  -p moving_alpha:="${RC_POSE_FILTER_STABLE_MOVING_ALPHA:-${RC_POSE_FILTER_MOVING_ALPHA:-0.92}}" \
  -p yaw_alpha:="${RC_POSE_FILTER_STABLE_YAW_ALPHA:-${RC_POSE_FILTER_YAW_ALPHA:-0.85}}" \
  -p static_speed_threshold:="${RC_POSE_FILTER_STABLE_STATIC_SPEED:-${RC_POSE_FILTER_STATIC_SPEED:-0.03}}" \
  -p full_speed_threshold:="${RC_POSE_FILTER_STABLE_FULL_SPEED:-${RC_POSE_FILTER_FULL_SPEED:-0.20}}" \
  -p static_yaw_rate_deg:="${RC_POSE_FILTER_STABLE_STATIC_YAW_RATE_DEG:-${RC_POSE_FILTER_STATIC_YAW_RATE_DEG:-8.0}}" \
  -p full_yaw_rate_deg:="${RC_POSE_FILTER_STABLE_FULL_YAW_RATE_DEG:-${RC_POSE_FILTER_FULL_YAW_RATE_DEG:-45.0}}" \
  -p spike_step_m:="${RC_POSE_FILTER_STABLE_SPIKE_STEP:-0.012}" \
  -p stable_confirm_samples:="${RC_POSE_FILTER_STABLE_CONFIRM_SAMPLES:-3}" \
  -p stable_return_tolerance_m:="${RC_POSE_FILTER_STABLE_RETURN_TOLERANCE:-0.006}" \
  -p stable_release_distance_m:="${RC_POSE_FILTER_STABLE_RELEASE_DISTANCE:-0.030}" \
  -p stable_window_size:="${RC_POSE_FILTER_STABLE_WINDOW_SIZE:-9}" \
  -p stable_moving_hold_sec:="${RC_POSE_FILTER_STABLE_MOVING_HOLD_SEC:-0.8}" \
  -p max_xy_jump:="${RC_POSE_FILTER_STABLE_MAX_XY_JUMP:-${RC_POSE_FILTER_MAX_XY_JUMP:-0.50}}" \
  -p max_xy_speed:="${RC_POSE_FILTER_STABLE_MAX_XY_SPEED:-${RC_POSE_FILTER_MAX_XY_SPEED:-5.0}}" \
  -p max_yaw_jump_deg:="${RC_POSE_FILTER_STABLE_MAX_YAW_JUMP_DEG:-${RC_POSE_FILTER_MAX_YAW_JUMP_DEG:-90.0}}" \
  -p recovery_reject_count:="${RC_POSE_FILTER_STABLE_RECOVERY_REJECT_COUNT:-${RC_POSE_FILTER_RECOVERY_REJECT_COUNT:-10}}" \
  -p recovery_time_sec:="${RC_POSE_FILTER_STABLE_RECOVERY_TIME:-${RC_POSE_FILTER_RECOVERY_TIME:-1.0}}"
