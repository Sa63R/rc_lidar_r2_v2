#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${RC_LIDAR_WORKSPACE:-$(cd "$DIR/.." && pwd)}"
LOG_DIR="${RC_LIDAR_LOG_ROOT:-$WORKSPACE_DIR/logs/autostart}"
BOOT_LOG="$LOG_DIR/xdg_autostart.log"
ATTEMPT_LOG="$LOG_DIR/boot_attempts.log"
LOCK_FILE="/tmp/rc_lidar_xdg_autostart.lock"
STATE_FILE="$LOG_DIR/boot_start_state.env"

mkdir -p "$LOG_DIR"

BOOT_ID="$(cat /proc/sys/kernel/random/boot_id 2>/dev/null || echo unknown)"
DRY_RUN="${RC_AUTOSTART_DRY_RUN:-0}"
FORCE_BOOT_START="${RC_LIDAR_FORCE_BOOT_START:-0}"

current_uptime_sec() {
  awk '{printf "%.3f", $1}' /proc/uptime 2>/dev/null || printf "0.000"
}

sync_file() {
  local path="$1"
  sync -f "$path" >/dev/null 2>&1 || sync >/dev/null 2>&1 || true
}

log_boot_event() {
  local message="$1"
  local uptime_sec
  uptime_sec="$(current_uptime_sec)"
  {
    echo "[$(date -Iseconds)] $message"
    echo "  boot_id: $BOOT_ID"
    echo "  uptime_sec: $uptime_sec"
    echo "  pid: $$"
    echo "  ppid: $PPID"
    echo "  display: ${DISPLAY:-}"
    echo "  xdg_session_type: ${XDG_SESSION_TYPE:-}"
  } >> "$BOOT_LOG" 2>&1
  {
    echo "$(date -Iseconds),boot_id=$BOOT_ID,uptime_sec=$uptime_sec,pid=$$,ppid=$PPID,message=$message"
  } >> "$ATTEMPT_LOG" 2>&1
  sync_file "$BOOT_LOG"
  sync_file "$ATTEMPT_LOG"
}

state_value() {
  local key="$1"
  if [ ! -f "$STATE_FILE" ]; then
    return 0
  fi
  awk -F= -v key="$key" '$1 == key {print substr($0, length(key) + 2)}' "$STATE_FILE" 2>/dev/null | tail -n 1 || true
}

write_state() {
  local status="$1"
  local uptime_sec
  uptime_sec="$(current_uptime_sec)"
  local tmp="$STATE_FILE.tmp.$$"
  {
    echo "boot_id=$BOOT_ID"
    echo "status=$status"
    echo "updated_at=$(date -Iseconds)"
    echo "uptime_sec=$uptime_sec"
    echo "pid=$$"
    echo "workspace=$WORKSPACE_DIR"
    echo "latest_log_dir=$(readlink -f "$LOG_DIR/latest" 2>/dev/null || true)"
  } > "$tmp"
  mv "$tmp" "$STATE_FILE"
  sync_file "$STATE_FILE"
}

{
  echo
  echo "========== XDG AUTOSTART =========="
  echo "time: $(date -Iseconds)"
  echo "boot_id: $BOOT_ID"
  echo "uptime_sec: $(current_uptime_sec)"
  echo "pid: $$"
  echo "user: $(whoami)"
  echo "display: ${DISPLAY:-}"
  echo "xdg_session_type: ${XDG_SESSION_TYPE:-}"
  echo "workspace: $WORKSPACE_DIR"
} >> "$BOOT_LOG" 2>&1
sync_file "$BOOT_LOG"
log_boot_event "boot_start invoked"

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  log_boot_event "another rc_lidar autostart is already running, skipped"
  exit 0
fi

previous_boot_id="$(state_value boot_id)"
previous_status="$(state_value status)"
if [ "$DRY_RUN" != "1" ] && [ "$FORCE_BOOT_START" != "1" ] && [ "$previous_boot_id" = "$BOOT_ID" ] && [ "$previous_status" != "failed" ]; then
  log_boot_event "same boot_id already handled with status=$previous_status, skipped"
  exit 0
fi

cd "$WORKSPACE_DIR"

DEFAULT_RELOCALIZATION_MAP="$WORKSPACE_DIR/maps/mid360_map_20260625_005422.pcd"

export RUN_REASON="${RUN_REASON:-boot}"
export RC_LIDAR_MODE="${RC_LIDAR_MODE:-relocalization}"
export RC_LOCALIZATION_MAP_PATH="${RC_LOCALIZATION_MAP_PATH:-${RC_LIDAR_MAP_PATH:-$DEFAULT_RELOCALIZATION_MAP}}"
export RC_LIDAR_MAP_PATH="$RC_LOCALIZATION_MAP_PATH"
export RC_RELOCALIZATION_ASSUME_MAP_ORIGIN="${RC_RELOCALIZATION_ASSUME_MAP_ORIGIN:-1}"
export RC_AUTOSTART_START_DELAY="${RC_AUTOSTART_START_DELAY:-15}"
export RC_AUTOSTART_HEADLESS="${RC_AUTOSTART_HEADLESS:-1}"
export RC_AUTOSTART_PLOTJUGGLER="${RC_AUTOSTART_PLOTJUGGLER:-1}"
export RC_AUTOSTART_CANFD="${RC_AUTOSTART_CANFD:-1}"
export RC_CAN_RESTART_LISTENER="${RC_CAN_RESTART_LISTENER:-auto}"
export RC_CANFD_RESTART_ENABLED="${RC_CANFD_RESTART_ENABLED:-1}"
export RC_CANFD_RESTART_CAN_ID="${RC_CANFD_RESTART_CAN_ID:-0x30}"
export RC_CANFD_RESTART_PAYLOAD_HEX="${RC_CANFD_RESTART_PAYLOAD_HEX:-A55A01FE00000000}"
export RC_CAN_RESTART_DEBOUNCE_SEC="${RC_CAN_RESTART_DEBOUNCE_SEC:-8.0}"
export RC_CAN_RESTART_MIN_INTERVAL_SEC="${RC_CAN_RESTART_MIN_INTERVAL_SEC:-15}"
export RC_CAN_DEBUG_CSV="${RC_CAN_DEBUG_CSV:-1}"
export FASTLIO_RVIZ="${FASTLIO_RVIZ:-true}"
export RC_CONTINUOUS_POSE="${RC_CONTINUOUS_POSE:-auto}"
export RC_CONTINUOUS_POSE_OUTPUT_TOPIC="${RC_CONTINUOUS_POSE_OUTPUT_TOPIC:-/robot/pose_continuous}"
export RC_CONTINUOUS_POSE_STAMP_MATCH_MODE="${RC_CONTINUOUS_POSE_STAMP_MATCH_MODE:-stamp}"
export RC_CONTINUOUS_POSE_MAX_STAMP_DELTA="${RC_CONTINUOUS_POSE_MAX_STAMP_DELTA:-0.03}"
export RC_CAN_POSE_TOPIC="${RC_CAN_POSE_TOPIC:-$RC_CONTINUOUS_POSE_OUTPUT_TOPIC}"
export RC_CAN_POSE_TIMEOUT_SEC="${RC_CAN_POSE_TIMEOUT_SEC:-0.5}"
export RC_CAN_RAW_ODOM_POSE_TOPIC="${RC_CAN_RAW_ODOM_POSE_TOPIC:-/Odometry}"
export RC_CAN_RAW_ODOM_POSE_TIMEOUT_SEC="${RC_CAN_RAW_ODOM_POSE_TIMEOUT_SEC:-0.5}"
export RC_CAN_RAW_ODOM_POSE_STRICT="${RC_CAN_RAW_ODOM_POSE_STRICT:-1}"
export RC_CAN_RAW_ODOM_POSE_YAW_OFFSET_RAD="${RC_CAN_RAW_ODOM_POSE_YAW_OFFSET_RAD:-0.0}"
export RC_CAN_USE_YAW_POSE_TOPIC="${RC_CAN_USE_YAW_POSE_TOPIC:-0}"
export RC_CAN_USE_RAW_ODOM_YAW="${RC_CAN_USE_RAW_ODOM_YAW:-0}"
export RC_CAN_RAW_ODOM_YAW_TOPIC="${RC_CAN_RAW_ODOM_YAW_TOPIC:-/Odometry}"
export RC_CAN_RAW_ODOM_YAW_STRICT="${RC_CAN_RAW_ODOM_YAW_STRICT:-0}"
export RC_CAN_RAW_ODOM_YAW_OFFSET_RAD="${RC_CAN_RAW_ODOM_YAW_OFFSET_RAD:-0.0}"
export RC_CAN_RAW_ODOM_YAW_IGNORE_READY_GATE="${RC_CAN_RAW_ODOM_YAW_IGNORE_READY_GATE:-1}"
export RC_CAN_REQUIRE_READY="${RC_CAN_REQUIRE_READY:-0}"
export RC_CAN_READY_QUALITY_TOPIC="${RC_CAN_READY_QUALITY_TOPIC:-/map_lidar/quality}"
export RC_CAN_READY_MAX_FITNESS="${RC_CAN_READY_MAX_FITNESS:-0.05}"
export RC_CAN_READY_MIN_ACCEPTS="${RC_CAN_READY_MIN_ACCEPTS:-5}"
export RC_POSE_FILTER="${RC_POSE_FILTER:-0}"
export RC_POSE_FILTER_SELECTED="${RC_POSE_FILTER_SELECTED:-normal}"
export RC_POSE_FILTER_BETA="${RC_POSE_FILTER_BETA:-0}"
export RC_POSE_FILTER_STABLE="${RC_POSE_FILTER_STABLE:-0}"
export RC_ROBOT_POSE_YAW_FUSION="${RC_ROBOT_POSE_YAW_FUSION:-0}"
export RC_HEALTH_MONITOR="${RC_HEALTH_MONITOR:-1}"
export RC_READY_NOTIFY="${RC_READY_NOTIFY:-1}"
export RC_READY_NOTIFY_POSE_TOPIC="${RC_READY_NOTIFY_POSE_TOPIC:-$RC_CONTINUOUS_POSE_OUTPUT_TOPIC}"
export RC_READY_NOTIFY_POPUP="${RC_READY_NOTIFY_POPUP:-0}"

if [ "$DRY_RUN" != "1" ]; then
  write_state "starting"
fi

set +e
./autostart/start_all_terminals.sh >> "$BOOT_LOG" 2>&1
rc=$?
set -e

if [ "$rc" -eq 0 ]; then
  log_boot_event "start_all_terminals finished successfully"
  if [ "$DRY_RUN" != "1" ]; then
    write_state "started"
  fi
else
  log_boot_event "start_all_terminals failed rc=$rc"
  if [ "$DRY_RUN" != "1" ]; then
    write_state "failed"
  fi
fi

exit "$rc"
