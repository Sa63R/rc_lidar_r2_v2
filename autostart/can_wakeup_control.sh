#!/usr/bin/env bash
set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${RC_LIDAR_WORKSPACE:-$(cd "$DIR/.." && pwd)}"
LOG_ROOT="${RC_LIDAR_LOG_ROOT:-$WORKSPACE_DIR/logs/autostart}"
WAKEUP_LOG_DIR="${RC_CAN_WAKEUP_LOG_DIR:-$LOG_ROOT/can_wakeup}"
CONTROL_LOCK="${RC_CAN_WAKEUP_CONTROL_LOCK:-/tmp/rc_can_wakeup_control.lock}"
GATEWAY_PID_FILE="${RC_CAN_WAKEUP_GATEWAY_PID_FILE:-/tmp/rc_can_wakeup_gateway.pid}"
LISTENER_PID_FILE="${RC_CAN_WAKEUP_LISTENER_PID_FILE:-/tmp/rc_can_wakeup_listener.pid}"
LISTENER_LOCK="${RC_CAN_WAKEUP_LISTENER_LOCK:-/tmp/rc_can_wakeup_listener.lock}"
HEARTBEAT_FILE="${RC_CAN_WAKEUP_HEARTBEAT_FILE:-/tmp/rc_can_wakeup_listener.heartbeat}"
EVENT_FILE="${RC_CAN_WAKEUP_EVENT_FILE:-/tmp/rc_can_wakeup_last_event}"
GATEWAY_SOCKET="${RC_FDCAN_GATEWAY_SOCKET:-/tmp/rc_fdcan_gateway.sock}"
GATEWAY_LOCK="${RC_FDCAN_GATEWAY_LOCK:-/tmp/rc_fdcan_gateway.lock}"
LISTENER_SCRIPT="$DIR/canfd_restart_listener.py"
GATEWAY_LOG="$WAKEUP_LOG_DIR/gateway.log"
LISTENER_LOG="$WAKEUP_LOG_DIR/restart_listener.log"
CONTROL_LOG="$WAKEUP_LOG_DIR/control.log"
HEARTBEAT_MAX_AGE_SEC="${RC_CAN_WAKEUP_HEARTBEAT_MAX_AGE_SEC:-3}"
START_TIMEOUT_SEC="${RC_CAN_WAKEUP_START_TIMEOUT_SEC:-8}"
STOP_TIMEOUT_SEC="${RC_CAN_WAKEUP_STOP_TIMEOUT_SEC:-3}"

mkdir -p "$WAKEUP_LOG_DIR"

log() {
  local message="$*"
  echo "[can_wakeup] $message"
  echo "[$(date -Iseconds)] $message" >> "$CONTROL_LOG"
}

read_pid() {
  cat "$1" 2>/dev/null || true
}

pid_is_gateway() {
  local pid="$1"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  local cmd stat
  stat="$(ps -p "$pid" -o stat= 2>/dev/null | tr -d ' ' || true)"
  case "$stat" in Z*|"") return 1 ;; esac
  cmd="$(ps -p "$pid" -o args= 2>/dev/null || true)"
  case "$cmd" in
    *"$DIR/fdcan_gateway.sh"*|*"ros2 run lidar_localization fdcan_gateway"*|*"/lidar_localization/fdcan_gateway"*) return 0 ;;
  esac
  return 1
}

pid_is_listener() {
  local pid="$1"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  local cmd stat
  stat="$(ps -p "$pid" -o stat= 2>/dev/null | tr -d ' ' || true)"
  case "$stat" in Z*|"") return 1 ;; esac
  cmd="$(ps -p "$pid" -o args= 2>/dev/null || true)"
  case "$cmd" in
    *"$LISTENER_SCRIPT"*) return 0 ;;
  esac
  return 1
}

find_gateway_pid() {
  local pid
  pid="$(read_pid "$GATEWAY_PID_FILE")"
  if pid_is_gateway "$pid"; then
    echo "$pid"
    return 0
  fi
  while IFS= read -r pid; do
    if pid_is_gateway "$pid"; then
      echo "$pid"
      return 0
    fi
  done < <(pgrep -f "$DIR/fdcan_gateway.sh|/lidar_localization/fdcan_gateway" 2>/dev/null || true)
  return 1
}

find_listener_pid() {
  local pid
  pid="$(read_pid "$LISTENER_PID_FILE")"
  if pid_is_listener "$pid"; then
    echo "$pid"
    return 0
  fi
  while IFS= read -r pid; do
    if pid_is_listener "$pid"; then
      echo "$pid"
      return 0
    fi
  done < <(pgrep -f "$LISTENER_SCRIPT" 2>/dev/null || true)
  return 1
}

gateway_probe() {
  /usr/bin/python3 "$LISTENER_SCRIPT" \
    --probe \
    --gateway-socket "$GATEWAY_SOCKET" \
    --probe-timeout-sec 0.25 >/dev/null 2>&1
}

heartbeat_is_fresh() {
  [ -f "$HEARTBEAT_FILE" ] || return 1
  local modified now age
  modified="$(stat -c %Y "$HEARTBEAT_FILE" 2>/dev/null || echo 0)"
  now="$(date +%s)"
  [[ "$modified" =~ ^[0-9]+$ ]] || return 1
  age=$((now - modified))
  [ "$age" -ge 0 ] && [ "$age" -le "$HEARTBEAT_MAX_AGE_SEC" ]
}

status_command() {
  local gateway_pid="" listener_pid=""
  local gateway_process="stopped" gateway_socket="down"
  local listener_process="stopped" listener_heartbeat="missing"
  local healthy=0 reason=""

  gateway_pid="$(find_gateway_pid || true)"
  if [ -n "$gateway_pid" ]; then
    gateway_process="running"
  fi
  if gateway_probe; then
    gateway_socket="ready"
  fi

  listener_pid="$(find_listener_pid || true)"
  if [ -n "$listener_pid" ]; then
    listener_process="running"
  fi
  if heartbeat_is_fresh; then
    listener_heartbeat="fresh"
  elif [ -f "$HEARTBEAT_FILE" ]; then
    listener_heartbeat="stale"
  fi

  if [ "$gateway_process" = "running" ] && [ "$gateway_socket" = "ready" ] && \
    [ "$listener_process" = "running" ] && [ "$listener_heartbeat" = "fresh" ]; then
    healthy=1
    reason="ok"
  else
    [ "$gateway_process" = "running" ] || reason="${reason}gateway_process,"
    [ "$gateway_socket" = "ready" ] || reason="${reason}gateway_socket,"
    [ "$listener_process" = "running" ] || reason="${reason}listener_process,"
    [ "$listener_heartbeat" = "fresh" ] || reason="${reason}listener_heartbeat,"
    reason="${reason%,}"
  fi

  echo "healthy=$healthy"
  echo "gateway_process=$gateway_process"
  echo "gateway_socket=$gateway_socket"
  echo "gateway_pid=$gateway_pid"
  echo "listener_process=$listener_process"
  echo "listener_heartbeat=$listener_heartbeat"
  echo "listener_pid=$listener_pid"
  echo "reason=$reason"
  if [ -f "$EVENT_FILE" ]; then
    while IFS= read -r event_line; do
      case "$event_line" in
        event_id=*|epoch=*|can_id=*|payload=*|action=*|detail=*)
          echo "last_event_${event_line}"
          ;;
      esac
    done < "$EVENT_FILE"
  fi
  [ "$healthy" = "1" ]
}

start_gateway() {
  local pid
  pid="$(find_gateway_pid || true)"
  if [ -n "$pid" ]; then
    echo "$pid" > "$GATEWAY_PID_FILE"
    log "Gateway already running pid=$pid"
    return 0
  fi

  # fdcan_gateway.sh sources the ROS environment itself.
  setsid "$DIR/fdcan_gateway.sh" 9>&- >> "$GATEWAY_LOG" 2>&1 < /dev/null &
  pid=$!
  echo "$pid" > "$GATEWAY_PID_FILE"
  log "Gateway started pid=$pid"
}

start_listener() {
  local pid
  pid="$(find_listener_pid || true)"
  if [ -n "$pid" ]; then
    if heartbeat_is_fresh; then
      echo "$pid" > "$LISTENER_PID_FILE"
      log "restart listener already running pid=$pid"
      return 0
    fi

    log "discarding stale restart listener pid=$pid without a fresh heartbeat"
    terminate_pid "$pid" listener TERM
    local deadline=$((SECONDS + 1))
    while pid_is_listener "$pid" && [ "$SECONDS" -lt "$deadline" ]; do
      sleep 0.1
    done
    if pid_is_listener "$pid"; then
      terminate_pid "$pid" listener KILL
    fi
    rm -f "$LISTENER_PID_FILE" "$HEARTBEAT_FILE"
  fi

  setsid /usr/bin/python3 -u "$LISTENER_SCRIPT" \
    --gateway-socket "$GATEWAY_SOCKET" \
    --can-id "${RC_CAN_RESTART_CAN_ID:-${RC_CANFD_RESTART_CAN_ID:-0x777}}" \
    --restart-script "${RC_RESTART_SCRIPT:-$DIR/restart_lidar_stack.sh}" \
    --debounce-sec "${RC_CAN_RESTART_DEBOUNCE_SEC:-8.0}" \
    --lock-path "$LISTENER_LOCK" \
    --heartbeat-path "$HEARTBEAT_FILE" \
    --event-path "$EVENT_FILE" \
    9>&- >> "$LISTENER_LOG" 2>&1 < /dev/null &
  pid=$!
  echo "$pid" > "$LISTENER_PID_FILE"
  log "restart listener started pid=$pid"
}

start_command() {
  exec 9>"$CONTROL_LOCK"
  flock -x 9
  start_gateway
  start_listener

  local deadline=$((SECONDS + START_TIMEOUT_SEC))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if status_command >/dev/null; then
      log "CAN wakeup chain is healthy"
      status_command
      return 0
    fi
    sleep 0.2
  done
  log "ERROR: CAN wakeup chain did not become healthy within ${START_TIMEOUT_SEC}s"
  status_command || true
  return 1
}

terminate_pid() {
  local pid="$1" kind="$2" signal="${3:-TERM}"
  if [ "$kind" = "gateway" ]; then
    pid_is_gateway "$pid" || return 0
  else
    pid_is_listener "$pid" || return 0
  fi
  local pgid
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  kill "-$signal" "$pid" 2>/dev/null || true
  if [ -n "$pgid" ] && [ "$pgid" != "1" ]; then
    kill "-$signal" -- "-$pgid" 2>/dev/null || true
  fi
}

stop_command() {
  exec 9>"$CONTROL_LOCK"
  flock -x 9
  local listener_pid gateway_pid deadline
  listener_pid="$(find_listener_pid || true)"
  gateway_pid="$(find_gateway_pid || true)"
  [ -z "$listener_pid" ] || terminate_pid "$listener_pid" listener TERM
  [ -z "$gateway_pid" ] || terminate_pid "$gateway_pid" gateway TERM

  deadline=$((SECONDS + STOP_TIMEOUT_SEC))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if [ -z "$(find_listener_pid || true)" ] && [ -z "$(find_gateway_pid || true)" ]; then
      break
    fi
    sleep 0.2
  done

  listener_pid="$(find_listener_pid || true)"
  gateway_pid="$(find_gateway_pid || true)"
  [ -z "$listener_pid" ] || terminate_pid "$listener_pid" listener KILL
  [ -z "$gateway_pid" ] || terminate_pid "$gateway_pid" gateway KILL
  rm -f "$GATEWAY_PID_FILE" "$LISTENER_PID_FILE" "$HEARTBEAT_FILE" "$EVENT_FILE" "$GATEWAY_SOCKET"
  rm -f /tmp/rc_fdcan_client_canfd_restart_listener_*.sock \
    /tmp/rc_fdcan_client_can_wakeup_probe_*.sock 2>/dev/null || true
  log "CAN wakeup chain stopped"
}

usage() {
  echo "Usage: $0 {start|stop|status}"
}

case "${1:-}" in
  start)
    start_command
    ;;
  stop)
    stop_command
    ;;
  status)
    status_command
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
