#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

case "${RC_LIDAR_MODE:-tracking}" in
  relocalization)
    ;;
  *)
    echo "[ready_notify] skipped because RC_LIDAR_MODE=${RC_LIDAR_MODE:-tracking}"
    exit 0
    ;;
esac

POSE_TOPIC="${RC_READY_NOTIFY_POSE_TOPIC:-${RC_CONTINUOUS_POSE_OUTPUT_TOPIC:-/robot/pose_continuous}}"
QUALITY_TOPIC="${RC_READY_NOTIFY_QUALITY_TOPIC:-/map_lidar/quality}"
MARKER_PATH="${RC_READY_NOTIFY_MARKER_PATH:-${LOG_DIR:-$RC_LIDAR_WORKSPACE/logs/autostart}/relocalization_ready.json}"
TIMEOUT_MARKER_PATH="${RC_READY_NOTIFY_TIMEOUT_MARKER_PATH:-${LOG_DIR:-$RC_LIDAR_WORKSPACE/logs/autostart}/relocalization_ready_timeout.json}"

echo "[ready_notify] pose_topic=$POSE_TOPIC"
echo "[ready_notify] quality_topic=$QUALITY_TOPIC"
echo "[ready_notify] marker_path=$MARKER_PATH"
echo "[ready_notify] stable_sec=${RC_READY_NOTIFY_STABLE_SEC:-3.0}"
echo "[ready_notify] timeout_sec=${RC_READY_NOTIFY_TIMEOUT_SEC:-120.0}"
echo "[ready_notify] max_fitness=${RC_READY_NOTIFY_MAX_FITNESS:-0.05}"

exec /usr/bin/env python3 -u "$RC_LIDAR_AUTOSTART/relocalization_ready_notify.py" \
  --pose-topic "$POSE_TOPIC" \
  --quality-topic "$QUALITY_TOPIC" \
  --marker-path "$MARKER_PATH" \
  --timeout-marker-path "$TIMEOUT_MARKER_PATH" \
  --stable-sec "${RC_READY_NOTIFY_STABLE_SEC:-3.0}" \
  --timeout-sec "${RC_READY_NOTIFY_TIMEOUT_SEC:-120.0}" \
  --max-topic-age "${RC_READY_NOTIFY_MAX_TOPIC_AGE:-2.0}" \
  --max-fitness "${RC_READY_NOTIFY_MAX_FITNESS:-0.05}" \
  --min-consecutive-accepts "${RC_READY_NOTIFY_MIN_ACCEPTS:-5}" \
  --window-seconds "${RC_READY_NOTIFY_WINDOW_SECONDS:-20}" \
  --popup "${RC_READY_NOTIFY_POPUP:-true}" \
  --popup-on-timeout "${RC_READY_NOTIFY_POPUP_ON_TIMEOUT:-false}"
