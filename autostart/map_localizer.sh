#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

to_bool() {
  case "${1:-false}" in
    1|true|TRUE|yes|YES|on|ON) echo "true" ;;
    *) echo "false" ;;
  esac
}

MAP_PATH="${RC_LOCALIZATION_MAP_PATH:-${RC_LIDAR_MAP_PATH:-$RC_LIDAR_WORKSPACE/maps/mid360_map.pcd}}"
export RC_LOCALIZATION_MAP_PATH="$MAP_PATH"
export RC_LIDAR_MAP_PATH="$MAP_PATH"
ASSUME_MAP_ORIGIN="$(to_bool "${RC_RELOCALIZATION_ASSUME_MAP_ORIGIN:-false}")"
INITIAL_GUESS="$(to_bool "${RC_RELOCALIZATION_INITIAL_GUESS:-false}")"
INITIAL_X="${RC_RELOCALIZATION_INITIAL_X:-0.0}"
INITIAL_Y="${RC_RELOCALIZATION_INITIAL_Y:-0.0}"
INITIAL_Z="${RC_RELOCALIZATION_INITIAL_Z:-0.0}"
INITIAL_ROLL="${RC_RELOCALIZATION_INITIAL_ROLL:-0.0}"
INITIAL_PITCH="${RC_RELOCALIZATION_INITIAL_PITCH:-0.0}"
INITIAL_YAW="${RC_RELOCALIZATION_INITIAL_YAW:-0.0}"
QUALITY_OVERLAP="$(to_bool "${RC_MAP_QUALITY_OVERLAP:-0}")"
QUALITY_OVERLAP_DISTANCE="${RC_MAP_QUALITY_OVERLAP_DISTANCE:-0.20}"
QUALITY_OVERLAP_PERIOD_SEC="${RC_MAP_QUALITY_OVERLAP_PERIOD_SEC:-1.0}"
QUALITY_OVERLAP_MAX_POINTS="${RC_MAP_QUALITY_OVERLAP_MAX_POINTS:-1000}"
TIMESTAMP_MISMATCH_WARN_SEC="${RC_MAP_TIMESTAMP_MISMATCH_WARN_SEC:-0.03}"
TIMESTAMP_MISMATCH_LOG_PERIOD_SEC="${RC_MAP_TIMESTAMP_MISMATCH_LOG_PERIOD_SEC:-2.0}"
INPUT_QUEUE_DEPTH="${RC_MAP_LOCALIZER_INPUT_QUEUE_DEPTH:-1}"
EXECUTOR_THREADS="${RC_MAP_LOCALIZER_EXECUTOR_THREADS:-3}"
ODOM_BUFFER_DURATION_SEC="${RC_MAP_LOCALIZER_ODOM_BUFFER_DURATION_SEC:-2.0}"
MAX_SCAN_ODOM_DELTA_SEC="${RC_MAP_LOCALIZER_MAX_SCAN_ODOM_DELTA_SEC:-0.05}"
MAX_SCAN_AGE_SEC="${RC_MAP_LOCALIZER_MAX_SCAN_AGE_SEC:-0.5}"
LOCALIZE_PERIOD_SEC="${RC_MAP_LOCALIZER_LOCALIZE_PERIOD_SEC:-0.2}"
MAP_LEAF_SIZE="${RC_MAP_LOCALIZER_MAP_LEAF_SIZE:-0.10}"
SCAN_LEAF_SIZE="${RC_MAP_LOCALIZER_SCAN_LEAF_SIZE:-0.10}"

if [ ! -s "$MAP_PATH" ]; then
  echo "[map_localizer] ERROR: map file is missing or empty: $MAP_PATH"
  echo "[map_localizer] Build and save a map first:"
  echo "[map_localizer]   RC_LIDAR_MODE=mapping ./autostart/start_all_terminals.sh"
  echo "[map_localizer]   ./autostart/save_fast_lio_map.sh"
  exit 3
fi

echo "[map_localizer] map_path=$MAP_PATH"
echo "[map_localizer] auto_initialize_identity=$ASSUME_MAP_ORIGIN"
echo "[map_localizer] initial_guess_enabled=$INITIAL_GUESS"
echo "[map_localizer] quality_overlap_enabled=$QUALITY_OVERLAP"
echo "[map_localizer] input_queue_depth=$INPUT_QUEUE_DEPTH"
echo "[map_localizer] executor_threads=$EXECUTOR_THREADS"
echo "[map_localizer] odom_buffer_duration_sec=$ODOM_BUFFER_DURATION_SEC"
echo "[map_localizer] max_scan_odom_delta_sec=$MAX_SCAN_ODOM_DELTA_SEC"
echo "[map_localizer] max_scan_age_sec=$MAX_SCAN_AGE_SEC"
echo "[map_localizer] localize_period_sec=$LOCALIZE_PERIOD_SEC"
echo "[map_localizer] map_leaf_size=$MAP_LEAF_SIZE"
echo "[map_localizer] scan_leaf_size=$SCAN_LEAF_SIZE"
echo "[map_localizer] timestamp_mismatch_warn_sec=$TIMESTAMP_MISMATCH_WARN_SEC"
if [ "$INITIAL_GUESS" = "true" ]; then
  echo "[map_localizer] initial map->camera_init guess xyz=($INITIAL_X, $INITIAL_Y, $INITIAL_Z) rpy=($INITIAL_ROLL, $INITIAL_PITCH, $INITIAL_YAW)"
fi

exec ros2 launch lidar_map_localization map_localizer.launch.py \
  map_path:="$MAP_PATH" \
  auto_initialize_identity:="$ASSUME_MAP_ORIGIN" \
  initial_guess_enabled:="$INITIAL_GUESS" \
  initial_guess_x:="$INITIAL_X" \
  initial_guess_y:="$INITIAL_Y" \
  initial_guess_z:="$INITIAL_Z" \
  initial_guess_roll:="$INITIAL_ROLL" \
  initial_guess_pitch:="$INITIAL_PITCH" \
  initial_guess_yaw:="$INITIAL_YAW" \
  quality_overlap_enabled:="$QUALITY_OVERLAP" \
  quality_overlap_distance:="$QUALITY_OVERLAP_DISTANCE" \
  quality_overlap_period_sec:="$QUALITY_OVERLAP_PERIOD_SEC" \
  quality_overlap_max_points:="$QUALITY_OVERLAP_MAX_POINTS" \
  timestamp_mismatch_warn_sec:="$TIMESTAMP_MISMATCH_WARN_SEC" \
  timestamp_mismatch_log_period_sec:="$TIMESTAMP_MISMATCH_LOG_PERIOD_SEC" \
  input_queue_depth:="$INPUT_QUEUE_DEPTH" \
  executor_threads:="$EXECUTOR_THREADS" \
  odom_buffer_duration_sec:="$ODOM_BUFFER_DURATION_SEC" \
  max_scan_odom_delta_sec:="$MAX_SCAN_ODOM_DELTA_SEC" \
  max_scan_age_sec:="$MAX_SCAN_AGE_SEC" \
  localize_period_sec:="$LOCALIZE_PERIOD_SEC" \
  map_leaf_size:="$MAP_LEAF_SIZE" \
  scan_leaf_size:="$SCAN_LEAF_SIZE"
