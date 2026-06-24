#!/usr/bin/env bash
set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$DIR/.." && pwd)"
LOG_ROOT="${RC_LIDAR_LOG_ROOT:-$WORKSPACE_DIR/logs/autostart}"
RESTART_LOG_DIR="$LOG_ROOT/restart"
mkdir -p "$RESTART_LOG_DIR"

RESTART_ID="$(date +%Y%m%d_%H%M%S)_can"
LOG="$RESTART_LOG_DIR/restart_${RESTART_ID}.log"
ln -sfn "$LOG" "$LOG_ROOT/latest_restart"

LOCK="/tmp/rc_lidar_restart.lock"

exec 9>"$LOCK"
flock -n 9 || exit 0

kill_pid_safely() {
  local pid="$1"

  if ! [[ "$pid" =~ ^[0-9]+$ ]]; then
    return 0
  fi

  if ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi

  echo "killing pid=$pid cmd=$(ps -p "$pid" -o args= 2>/dev/null || true)"

  local self_pgid
  self_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d ' ')"

  local target_pgid
  target_pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"

  # 先杀这个窗口里启动的子进程
  pkill -TERM -P "$pid" 2>/dev/null || true
  kill -TERM "$pid" 2>/dev/null || true

  # 如果它有独立进程组，杀整个进程组，防止 ros2 launch 子进程残留
  if [ -n "${target_pgid:-}" ] && [ "$target_pgid" != "$self_pgid" ] && [ "$target_pgid" != "1" ]; then
    kill -TERM -- "-$target_pgid" 2>/dev/null || true
  fi
}

force_kill_pid_safely() {
  local pid="$1"

  if ! [[ "$pid" =~ ^[0-9]+$ ]]; then
    return 0
  fi

  if ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi

  echo "force killing pid=$pid cmd=$(ps -p "$pid" -o args= 2>/dev/null || true)"

  local self_pgid
  self_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d ' ')"

  local target_pgid
  target_pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"

  pkill -KILL -P "$pid" 2>/dev/null || true
  kill -KILL "$pid" 2>/dev/null || true

  if [ -n "${target_pgid:-}" ] && [ "$target_pgid" != "$self_pgid" ] && [ "$target_pgid" != "1" ]; then
    kill -KILL -- "-$target_pgid" 2>/dev/null || true
  fi
}

close_terminal_windows_by_title() {
  local titles=(
    "LIVOX"
    "FASTLIO"
    "CAN_SERIAL"
    "CAN+SERIAL"
    "LIDAR_XYZR"
    "TF_STATIC1"
    "TF_STATIC2"
    "TF_STATIC3"
    "HIGHRATE_BODY"
    "world_tf"
    "TF_ROBOT_OFFSET"
    "TF_LIDAR_OFFSET"
    "PLOTJUGGLER"
    "MAP_LOCALIZER"
    "POSE_INTERFACE"
    "POSE_FILTER"
    "POSE_FILTER_BETA"
    "HEALTH_MONITOR"
    "THERMAL_MONITOR"
    "READY_NOTIFY"
    "RC LiDAR Ready"
    "RC LiDAR Not Ready"
  )

  if command -v wmctrl >/dev/null 2>&1; then
    for title in "${titles[@]}"; do
      wmctrl -c "$title" 2>/dev/null || true
    done
  fi

  if command -v xdotool >/dev/null 2>&1; then
    for title in "${titles[@]}"; do
      xdotool search --name "$title" windowkill 2>/dev/null || true
    done
  fi
}

{
  echo
  echo "========== restart requested at $(date '+%F %T') =========="

  sleep 0.5

  echo "[1/5] stopping old terminal child processes by pid files..."

  shopt -s nullglob
  pidfiles=(/tmp/rc_autostart_*.pid)

  for f in "${pidfiles[@]}"; do
    pid="$(cat "$f" 2>/dev/null || true)"
    echo "pidfile=$f pid=$pid"
    kill_pid_safely "$pid"
  done

  echo "[2/5] stopping old processes by fallback patterns..."

  patterns=(
    "$DIR/livox.sh"
    "$DIR/fastlio.sh"
    "$DIR/serial_driver.sh"
    "$DIR/lidar_xyzr.sh"
    "$DIR/lidar_to_base_tf.sh"
    "$DIR/map_to_init_tf.sh"
    "$DIR/lidar_to_base_highrate_tf.sh"
    "$DIR/highrate_body.sh"
    "$DIR/world_tf.sh"
    "$DIR/robot_tf_offsets.sh"
    "$DIR/lidar_tf_offset.sh"
    "$DIR/plotjuggler_tf_compare.sh"
    "$DIR/map_localizer.sh"
    "$DIR/pose_interface_mapping.sh"
    "$DIR/pose_interface_relocalization.sh"
    "$DIR/pose_stability_filter.sh"
    "$DIR/pose_stability_filter_beta.sh"
    "$DIR/thermal_pose_monitor.sh"
    "$DIR/relocalization_ready_notify.sh"
    "ros2 launch rc_serial_driver serial_driver.launch.py"
    "ros2 launch lidar_map_localization"
    "ros2 launch lidar_localization"
    "map_localizer"
    "pose_from_odom"
    "pose_stability_filter"
    "thermal_pose_monitor.py"
    "relocalization_ready_notify.py"
    "$DIR/diagnose_logger.sh"
  )

  for p in "${patterns[@]}"; do
    pkill -TERM -f "$p" 2>/dev/null || true
  done

  close_terminal_windows_by_title

  sleep 2

  echo "[3/5] force killing remaining old processes if needed..."

  for f in "${pidfiles[@]}"; do
    pid="$(cat "$f" 2>/dev/null || true)"
    force_kill_pid_safely "$pid"
  done

  for p in "${patterns[@]}"; do
    pkill -KILL -f "$p" 2>/dev/null || true
  done

  close_terminal_windows_by_title

  rm -f /tmp/rc_autostart_*.pid 2>/dev/null || true

  echo "[4/5] starting new stack..."
  cd "$DIR"

  RUN_REASON="can_restart" \
    RC_LIDAR_MODE="${RC_LIDAR_MODE:-relocalization}" \
    RC_RELOCALIZATION_ASSUME_MAP_ORIGIN="${RC_RELOCALIZATION_ASSUME_MAP_ORIGIN:-1}" \
    RC_AUTOSTART_HEADLESS="${RC_AUTOSTART_HEADLESS:-1}" \
    RC_AUTOSTART_PLOTJUGGLER="${RC_AUTOSTART_PLOTJUGGLER:-0}" \
    RC_POSE_FILTER_SELECTED="${RC_POSE_FILTER_SELECTED:-normal}" \
    RC_POSE_FILTER_BETA="${RC_POSE_FILTER_BETA:-0}" \
    RC_BETA_FILTERED_POSE_TOPIC="${RC_BETA_FILTERED_POSE_TOPIC:-/robot/pose_filtered_beta}" \
    RC_CAN_POSE_SOURCE="${RC_CAN_POSE_SOURCE:-pose_topic}" \
    RC_HEALTH_MONITOR="${RC_HEALTH_MONITOR:-1}" \
    RESTART_LOG="$LOG" \
    nohup "$DIR/start_all_terminals.sh" >> "$LOG" 2>&1 &

  echo "[5/5] restart command launched."
  echo "========== done at $(date '+%F %T') =========="
} >> "$LOG" 2>&1
