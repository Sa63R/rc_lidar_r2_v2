#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="/home/orangepi/RC_LIDAR_NOW"
LOG_DIR="$WORKSPACE_DIR/logs/autostart"
BOOT_LOG="$LOG_DIR/xdg_autostart.log"
LOCK_FILE="/tmp/rc_lidar_xdg_autostart.lock"

mkdir -p "$LOG_DIR"

{
  echo
  echo "========== XDG AUTOSTART =========="
  echo "time: $(date -Iseconds)"
  echo "user: $(whoami)"
  echo "display: ${DISPLAY:-}"
  echo "xdg_session_type: ${XDG_SESSION_TYPE:-}"
  echo "workspace: $WORKSPACE_DIR"
} >> "$BOOT_LOG" 2>&1

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "[boot] another rc_lidar autostart is already running, skipped." >> "$BOOT_LOG" 2>&1
  exit 0
fi

cd "$WORKSPACE_DIR"

export RUN_REASON="${RUN_REASON:-boot}"
export RC_LIDAR_MODE="${RC_LIDAR_MODE:-relocalization}"
export RC_RELOCALIZATION_ASSUME_MAP_ORIGIN="${RC_RELOCALIZATION_ASSUME_MAP_ORIGIN:-1}"
export RC_AUTOSTART_START_DELAY="${RC_AUTOSTART_START_DELAY:-15}"
export RC_AUTOSTART_HEADLESS="${RC_AUTOSTART_HEADLESS:-1}"
export RC_AUTOSTART_PLOTJUGGLER="${RC_AUTOSTART_PLOTJUGGLER:-0}"
export RC_CAN_POSE_SOURCE="${RC_CAN_POSE_SOURCE:-pose_topic}"
export RC_CAN_POSE_TOPIC="${RC_CAN_POSE_TOPIC:-/robot/pose_filtered}"
export RC_HEALTH_MONITOR="${RC_HEALTH_MONITOR:-1}"

./autostart/start_all_terminals.sh >> "$BOOT_LOG" 2>&1
