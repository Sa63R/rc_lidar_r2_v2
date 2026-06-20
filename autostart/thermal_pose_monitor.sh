#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

OUTPUT_DIR="${RC_HEALTH_MONITOR_OUTPUT_DIR:-${RC_THERMAL_POSE_MONITOR_OUTPUT_DIR:-${LOG_DIR:-$RC_LIDAR_WORKSPACE/logs/health_monitor}}}"
INTERVAL="${RC_HEALTH_MONITOR_INTERVAL:-${RC_THERMAL_POSE_MONITOR_INTERVAL:-1.0}}"
WINDOW="${RC_HEALTH_MONITOR_WINDOW:-${RC_THERMAL_POSE_MONITOR_WINDOW:-10.0}}"
SUMMARY_INTERVAL="${RC_HEALTH_MONITOR_SUMMARY_INTERVAL:-${RC_THERMAL_POSE_MONITOR_SUMMARY_INTERVAL:-30.0}}"
RAW_TOPIC="${RC_HEALTH_MONITOR_RAW_TOPIC:-${RC_THERMAL_POSE_MONITOR_RAW_TOPIC:-/robot/pose}}"
FILTERED_TOPIC="${RC_HEALTH_MONITOR_FILTERED_TOPIC:-${RC_THERMAL_POSE_MONITOR_FILTERED_TOPIC:-/robot/pose_filtered}}"
QUALITY_TOPIC="${RC_HEALTH_MONITOR_QUALITY_TOPIC:-${RC_THERMAL_POSE_MONITOR_QUALITY_TOPIC:-/map_lidar/quality}}"
STATIC_SPEED="${RC_HEALTH_MONITOR_STATIC_SPEED:-0.015}"
FULL_SPEED="${RC_HEALTH_MONITOR_FULL_SPEED:-0.12}"
STATIC_YAW_RATE_DEG="${RC_HEALTH_MONITOR_STATIC_YAW_RATE_DEG:-5.0}"
FULL_YAW_RATE_DEG="${RC_HEALTH_MONITOR_FULL_YAW_RATE_DEG:-35.0}"
STATIONARY_FACTOR="${RC_HEALTH_MONITOR_STATIONARY_FACTOR:-0.25}"
MAX_POSE_AGE="${RC_HEALTH_MONITOR_MAX_POSE_AGE:-2.0}"
RETENTION_DAYS="${RC_HEALTH_MONITOR_RETENTION_DAYS:-7}"

mkdir -p "$OUTPUT_DIR"

if [[ "$RETENTION_DAYS" =~ ^[0-9]+$ ]] && [ "$RETENTION_DAYS" -gt 0 ]; then
  find "$RC_LIDAR_WORKSPACE/logs/autostart" \
    -path "*/health_monitor.csv" -mtime +"$RETENTION_DAYS" -delete 2>/dev/null || true
  find "$RC_LIDAR_WORKSPACE/logs/autostart" \
    -path "*/health_monitor_summary.json" -mtime +"$RETENTION_DAYS" -delete 2>/dev/null || true
fi

echo "[health_monitor] output_dir=$OUTPUT_DIR"
echo "[health_monitor] interval=$INTERVAL window=$WINDOW summary_interval=$SUMMARY_INTERVAL"
echo "[health_monitor] raw_topic=$RAW_TOPIC"
echo "[health_monitor] filtered_topic=$FILTERED_TOPIC"
echo "[health_monitor] quality_topic=$QUALITY_TOPIC"
echo "[health_monitor] static_speed=$STATIC_SPEED full_speed=$FULL_SPEED static_yaw_rate_deg=$STATIC_YAW_RATE_DEG full_yaw_rate_deg=$FULL_YAW_RATE_DEG"

exec /usr/bin/env python3 -u "$RC_LIDAR_AUTOSTART/thermal_pose_monitor.py" \
  --output-dir "$OUTPUT_DIR" \
  --interval "$INTERVAL" \
  --window "$WINDOW" \
  --summary-interval "$SUMMARY_INTERVAL" \
  --raw-topic "$RAW_TOPIC" \
  --filtered-topic "$FILTERED_TOPIC" \
  --quality-topic "$QUALITY_TOPIC" \
  --static-speed "$STATIC_SPEED" \
  --full-speed "$FULL_SPEED" \
  --static-yaw-rate-deg "$STATIC_YAW_RATE_DEG" \
  --full-yaw-rate-deg "$FULL_YAW_RATE_DEG" \
  --stationary-factor "$STATIONARY_FACTOR" \
  --max-pose-age "$MAX_POSE_AGE"
