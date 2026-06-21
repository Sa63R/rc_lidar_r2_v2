#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

OUTPUT_DIR="${RC_HEALTH_MONITOR_OUTPUT_DIR:-${RC_THERMAL_POSE_MONITOR_OUTPUT_DIR:-${LOG_DIR:-$RC_LIDAR_WORKSPACE/logs/health_monitor}}}"
INTERVAL="${RC_HEALTH_MONITOR_INTERVAL:-${RC_THERMAL_POSE_MONITOR_INTERVAL:-0.2}}"
WINDOW="${RC_HEALTH_MONITOR_WINDOW:-${RC_THERMAL_POSE_MONITOR_WINDOW:-10.0}}"
SUMMARY_INTERVAL="${RC_HEALTH_MONITOR_SUMMARY_INTERVAL:-${RC_THERMAL_POSE_MONITOR_SUMMARY_INTERVAL:-30.0}}"
FSYNC_INTERVAL="${RC_HEALTH_MONITOR_FSYNC_INTERVAL:-${RC_THERMAL_POSE_MONITOR_FSYNC_INTERVAL:-5.0}}"
RAW_TOPIC="${RC_HEALTH_MONITOR_RAW_TOPIC:-${RC_THERMAL_POSE_MONITOR_RAW_TOPIC:-/robot/pose}}"
FILTERED_TOPIC="${RC_HEALTH_MONITOR_FILTERED_TOPIC:-${RC_THERMAL_POSE_MONITOR_FILTERED_TOPIC:-/robot/pose_filtered}}"
QUALITY_TOPIC="${RC_HEALTH_MONITOR_QUALITY_TOPIC:-${RC_THERMAL_POSE_MONITOR_QUALITY_TOPIC:-/map_lidar/quality}}"
FILTER_STATUS_TOPIC="${RC_HEALTH_MONITOR_FILTER_STATUS_TOPIC:-/lidar/pose_filter/status}"
STATIC_SPEED="${RC_HEALTH_MONITOR_STATIC_SPEED:-0.015}"
FULL_SPEED="${RC_HEALTH_MONITOR_FULL_SPEED:-0.12}"
STATIC_YAW_RATE_DEG="${RC_HEALTH_MONITOR_STATIC_YAW_RATE_DEG:-5.0}"
FULL_YAW_RATE_DEG="${RC_HEALTH_MONITOR_FULL_YAW_RATE_DEG:-35.0}"
STATIONARY_FACTOR="${RC_HEALTH_MONITOR_STATIONARY_FACTOR:-0.25}"
MAX_POSE_AGE="${RC_HEALTH_MONITOR_MAX_POSE_AGE:-2.0}"
EVENT_PRE_SEC="${RC_HEALTH_MONITOR_EVENT_PRE_SEC:-3.0}"
EVENT_POST_SEC="${RC_HEALTH_MONITOR_EVENT_POST_SEC:-3.0}"
EVENT_CONTEXT_CSV="${RC_HEALTH_MONITOR_EVENT_CONTEXT_CSV:-0}"
SPIKE_RAW_STEP="${RC_HEALTH_MONITOR_SPIKE_RAW_STEP:-0.02}"
SPIKE_FILTERED_STEP="${RC_HEALTH_MONITOR_SPIKE_FILTERED_STEP:-0.015}"
SPIKE_MAP_JUMP="${RC_HEALTH_MONITOR_SPIKE_MAP_JUMP:-0.02}"
SPIKE_FITNESS="${RC_HEALTH_MONITOR_SPIKE_FITNESS:-0.02}"
SUSTAINED_MOTION_SEC="${RC_HEALTH_MONITOR_SUSTAINED_MOTION_SEC:-2.0}"
RETENTION_DAYS="${RC_HEALTH_MONITOR_RETENTION_DAYS:-7}"

mkdir -p "$OUTPUT_DIR"

if [[ "$RETENTION_DAYS" =~ ^[0-9]+$ ]] && [ "$RETENTION_DAYS" -gt 0 ]; then
  find "$RC_LIDAR_WORKSPACE/logs/autostart" \
    -path "*/health_monitor.csv" -mtime +"$RETENTION_DAYS" -delete 2>/dev/null || true
  find "$RC_LIDAR_WORKSPACE/logs/autostart" \
    -path "*/health_monitor_summary.json" -mtime +"$RETENTION_DAYS" -delete 2>/dev/null || true
  find "$RC_LIDAR_WORKSPACE/logs/autostart" \
    -path "*/health_monitor_run.json" -mtime +"$RETENTION_DAYS" -delete 2>/dev/null || true
  find "$RC_LIDAR_WORKSPACE/logs/autostart" \
    -path "*/health_monitor_stop.json" -mtime +"$RETENTION_DAYS" -delete 2>/dev/null || true
  find "$RC_LIDAR_WORKSPACE/logs/autostart" \
    -path "*/health_events.jsonl" -mtime +"$RETENTION_DAYS" -delete 2>/dev/null || true
  find "$RC_LIDAR_WORKSPACE/logs/autostart" \
    -path "*/health_event_context.csv" -mtime +"$RETENTION_DAYS" -delete 2>/dev/null || true
fi

echo "[health_monitor] output_dir=$OUTPUT_DIR"
echo "[health_monitor] interval=$INTERVAL window=$WINDOW summary_interval=$SUMMARY_INTERVAL fsync_interval=$FSYNC_INTERVAL"
echo "[health_monitor] raw_topic=$RAW_TOPIC"
echo "[health_monitor] filtered_topic=$FILTERED_TOPIC"
echo "[health_monitor] quality_topic=$QUALITY_TOPIC"
echo "[health_monitor] filter_status_topic=$FILTER_STATUS_TOPIC"
echo "[health_monitor] events=health_events.jsonl pre=${EVENT_PRE_SEC}s post=${EVENT_POST_SEC}s context_csv=$EVENT_CONTEXT_CSV"
echo "[health_monitor] spike raw=$SPIKE_RAW_STEP filtered=$SPIKE_FILTERED_STEP map_jump=$SPIKE_MAP_JUMP fitness=$SPIKE_FITNESS sustained_motion=${SUSTAINED_MOTION_SEC}s"
echo "[health_monitor] static_speed=$STATIC_SPEED full_speed=$FULL_SPEED static_yaw_rate_deg=$STATIC_YAW_RATE_DEG full_yaw_rate_deg=$FULL_YAW_RATE_DEG"

EXTRA_ARGS=()
case "$EVENT_CONTEXT_CSV" in
  1|true|TRUE|yes|YES|on|ON) EXTRA_ARGS+=(--event-context-csv) ;;
esac

exec /usr/bin/env python3 -u "$RC_LIDAR_AUTOSTART/thermal_pose_monitor.py" \
  --output-dir "$OUTPUT_DIR" \
  --interval "$INTERVAL" \
  --window "$WINDOW" \
  --summary-interval "$SUMMARY_INTERVAL" \
  --fsync-interval "$FSYNC_INTERVAL" \
  --raw-topic "$RAW_TOPIC" \
  --filtered-topic "$FILTERED_TOPIC" \
  --quality-topic "$QUALITY_TOPIC" \
  --filter-status-topic "$FILTER_STATUS_TOPIC" \
  --static-speed "$STATIC_SPEED" \
  --full-speed "$FULL_SPEED" \
  --static-yaw-rate-deg "$STATIC_YAW_RATE_DEG" \
  --full-yaw-rate-deg "$FULL_YAW_RATE_DEG" \
  --stationary-factor "$STATIONARY_FACTOR" \
  --max-pose-age "$MAX_POSE_AGE" \
  --event-pre-sec "$EVENT_PRE_SEC" \
  --event-post-sec "$EVENT_POST_SEC" \
  --spike-raw-step "$SPIKE_RAW_STEP" \
  --spike-filtered-step "$SPIKE_FILTERED_STEP" \
  --spike-map-jump "$SPIKE_MAP_JUMP" \
  --spike-fitness "$SPIKE_FITNESS" \
  --sustained-motion-sec "$SUSTAINED_MOTION_SEC" \
  "${EXTRA_ARGS[@]}"
