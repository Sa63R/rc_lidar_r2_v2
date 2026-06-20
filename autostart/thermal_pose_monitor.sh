#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

OUTPUT_DIR="${RC_THERMAL_POSE_MONITOR_OUTPUT_DIR:-${LOG_DIR:-$RC_LIDAR_WORKSPACE/logs/thermal_pose_monitor}}"
INTERVAL="${RC_THERMAL_POSE_MONITOR_INTERVAL:-1.0}"
WINDOW="${RC_THERMAL_POSE_MONITOR_WINDOW:-10.0}"
SUMMARY_INTERVAL="${RC_THERMAL_POSE_MONITOR_SUMMARY_INTERVAL:-30.0}"
RAW_TOPIC="${RC_THERMAL_POSE_MONITOR_RAW_TOPIC:-/robot/pose}"
FILTERED_TOPIC="${RC_THERMAL_POSE_MONITOR_FILTERED_TOPIC:-/robot/pose_filtered}"
QUALITY_TOPIC="${RC_THERMAL_POSE_MONITOR_QUALITY_TOPIC:-/map_lidar/quality}"

mkdir -p "$OUTPUT_DIR"

echo "[thermal_pose_monitor] output_dir=$OUTPUT_DIR"
echo "[thermal_pose_monitor] interval=$INTERVAL window=$WINDOW summary_interval=$SUMMARY_INTERVAL"
echo "[thermal_pose_monitor] raw_topic=$RAW_TOPIC"
echo "[thermal_pose_monitor] filtered_topic=$FILTERED_TOPIC"
echo "[thermal_pose_monitor] quality_topic=$QUALITY_TOPIC"

exec /usr/bin/env python3 -u "$RC_LIDAR_AUTOSTART/thermal_pose_monitor.py" \
  --output-dir "$OUTPUT_DIR" \
  --interval "$INTERVAL" \
  --window "$WINDOW" \
  --summary-interval "$SUMMARY_INTERVAL" \
  --raw-topic "$RAW_TOPIC" \
  --filtered-topic "$FILTERED_TOPIC" \
  --quality-topic "$QUALITY_TOPIC"
