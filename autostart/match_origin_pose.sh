#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

INPUT_TOPIC="${RC_MATCH_ORIGIN_INPUT_TOPIC:-${RC_CONTINUOUS_POSE_OUTPUT_TOPIC:-/robot/pose_continuous}}"
INPUT_STATUS_TOPIC="${RC_MATCH_ORIGIN_INPUT_STATUS_TOPIC:-${RC_CONTINUOUS_POSE_STATUS_TOPIC:-/robot/pose_continuous/status}}"
OUTPUT_TOPIC="${RC_MATCH_ORIGIN_OUTPUT_TOPIC:-/robot/pose_match}"
OUTPUT_ODOM_TOPIC="${RC_MATCH_ORIGIN_OUTPUT_ODOM_TOPIC:-/robot/odom_match}"
STATUS_TOPIC="${RC_MATCH_ORIGIN_STATUS_TOPIC:-/robot/match_origin/status}"

bool_param() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON)
      printf 'true'
      ;;
    0|false|FALSE|no|NO|off|OFF|'')
      printf 'false'
      ;;
    *)
      printf '%s' "$1"
      ;;
  esac
}

REQUIRE_CORRECTION_READY="$(bool_param "${RC_MATCH_ORIGIN_REQUIRE_CORRECTION_READY:-true}")"
USE_QUALITY_GATE="$(bool_param "${RC_MATCH_ORIGIN_USE_QUALITY_GATE:-false}")"

echo "[match_origin] input_pose=$INPUT_TOPIC"
echo "[match_origin] input_status=$INPUT_STATUS_TOPIC"
echo "[match_origin] output_pose=$OUTPUT_TOPIC output_odom=$OUTPUT_ODOM_TOPIC"
echo "[match_origin] status=$STATUS_TOPIC"
echo "[match_origin] min_lock_sec=${RC_MATCH_ORIGIN_MIN_LOCK_SEC:-2.5} min_samples=${RC_MATCH_ORIGIN_MIN_SAMPLES:-20}"
echo "[match_origin] max_xy_std=${RC_MATCH_ORIGIN_MAX_XY_STD:-0.08} max_yaw_std_deg=${RC_MATCH_ORIGIN_MAX_YAW_STD_DEG:-5.0}"
echo "[match_origin] require_correction_ready=$REQUIRE_CORRECTION_READY use_quality_gate=$USE_QUALITY_GATE fitness_warn=${RC_MATCH_ORIGIN_FITNESS_WARN:-0.30}"

exec ros2 run lidar_localization match_origin_pose --ros-args \
  -p input_pose_topic:="$INPUT_TOPIC" \
  -p input_status_topic:="$INPUT_STATUS_TOPIC" \
  -p output_pose_topic:="$OUTPUT_TOPIC" \
  -p output_odom_topic:="$OUTPUT_ODOM_TOPIC" \
  -p status_topic:="$STATUS_TOPIC" \
  -p output_frame_id:="${RC_MATCH_ORIGIN_FRAME_ID:-match_origin}" \
  -p child_frame_id:="${RC_MATCH_ORIGIN_CHILD_FRAME_ID:-robot_base}" \
  -p min_lock_sec:="${RC_MATCH_ORIGIN_MIN_LOCK_SEC:-2.5}" \
  -p min_samples:="${RC_MATCH_ORIGIN_MIN_SAMPLES:-20}" \
  -p max_xy_std:="${RC_MATCH_ORIGIN_MAX_XY_STD:-0.08}" \
  -p max_yaw_std_deg:="${RC_MATCH_ORIGIN_MAX_YAW_STD_DEG:-5.0}" \
  -p require_correction_ready:="$REQUIRE_CORRECTION_READY" \
  -p status_timeout_sec:="${RC_MATCH_ORIGIN_STATUS_TIMEOUT_SEC:-2.0}" \
  -p use_quality_gate:="$USE_QUALITY_GATE" \
  -p fitness_warn:="${RC_MATCH_ORIGIN_FITNESS_WARN:-0.30}" \
  -p log_period_sec:="${RC_MATCH_ORIGIN_LOG_PERIOD_SEC:-1.0}"
