#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$DIR/.." && pwd)"
LOG_ROOT="${RC_LIDAR_LOG_ROOT:-$WORKSPACE_DIR/logs/autostart}"

# manual：手动启动
# boot：开机启动
# can_restart：CAN 报文触发重启
RUN_REASON="${RUN_REASON:-manual}"

RUN_ID="$(date +%Y%m%d_%H%M%S)_${RUN_REASON}"
LOG_DIR="$LOG_ROOT/run_$RUN_ID"

mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

export RUN_ID
export LOG_DIR
export RUN_REASON

# ROS2 自己的日志目录
export ROS_LOG_DIR="$LOG_DIR/ros2"
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0

mkdir -p "$ROS_LOG_DIR"

{
  echo "========== START ALL TERMINALS =========="
  echo "time: $(date -Iseconds)"
  echo "run_id: $RUN_ID"
  echo "run_reason: $RUN_REASON"
  echo "log_dir: $LOG_DIR"
  echo "dir: $DIR"
  echo "user: $(whoami)"
  echo "host: $(hostname)"
  echo "DISPLAY: ${DISPLAY:-}"
  echo "XDG_SESSION_TYPE: ${XDG_SESSION_TYPE:-}"
  echo "ROS_LOG_DIR: $ROS_LOG_DIR"
  echo "========================================="
} | tee "$LOG_DIR/start_summary.txt"

# 如果是 restart_lidar_stack.sh 调起来的，记录来源
if [ -n "${RESTART_LOG:-}" ]; then
  {
    echo "restart_log: $RESTART_LOG"
    echo "restart_time: $(date -Iseconds)"
    echo "run_reason: $RUN_REASON"
  } > "$LOG_DIR/restart_source.txt"
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "[ERROR] gnome-terminal not found. Install it or run scripts manually." | tee -a "$LOG_DIR/start_summary.txt"
  exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
  echo "[ERROR] DISPLAY is empty. start_all_terminals.sh needs a graphical session." | tee -a "$LOG_DIR/start_summary.txt"
  exit 1
fi

# 开机启动时桌面可能还没完全起来，手动启动多等几秒也没坏处
if [ "${RC_AUTOSTART_DRY_RUN:-0}" = "1" ]; then
  START_DELAY=0
else
  START_DELAY="${RC_AUTOSTART_START_DELAY:-3}"
fi
sleep "$START_DELAY"

# 可选后台健康检查：没有这个脚本也不影响启动
if [ -x "$DIR/diagnose_logger.sh" ]; then
  "$DIR/diagnose_logger.sh" "$LOG_DIR" > "$LOG_DIR/diagnose_logger.out" 2>&1 &
  echo $! > /tmp/rc_autostart_DIAGNOSE.pid
else
  echo "diagnose_logger.sh not found or not executable, skipped." > "$LOG_DIR/diagnose_logger.out"
fi

start_one() {
  local title="$1"
  local script="$2"
  local pidfile="/tmp/rc_autostart_${title}.pid"
  local logfile="$LOG_DIR/${title}.log"
  local statusfile="$LOG_DIR/${title}.status"

  {
    echo "[$(date -Iseconds)] launching $title -> $script"
    echo "logfile: $logfile"
    echo "pidfile: $pidfile"
  } >> "$LOG_DIR/start_summary.txt"

  if [ "${RC_AUTOSTART_DRY_RUN:-0}" = "1" ]; then
    echo "[dry-run] would launch $title using $DIR/$script" | tee -a "$LOG_DIR/start_summary.txt"
    return 0
  fi

  gnome-terminal --title="$title" -- bash -lc "
    echo \$\$ > '$pidfile'

    export RUN_ID='$RUN_ID'
    export LOG_DIR='$LOG_DIR'
    export RUN_REASON='$RUN_REASON'
    export ROS_LOG_DIR='$ROS_LOG_DIR'
    export RCUTILS_LOGGING_USE_STDOUT=1
    export RCUTILS_LOGGING_BUFFERED_STREAM=0

    {
      echo '========== TERMINAL START =========='
      echo 'time: '\"\$(date -Iseconds)\"
      echo 'title: $title'
      echo 'script: $DIR/$script'
      echo 'pidfile: $pidfile'
      echo 'logfile: $logfile'
      echo 'log_dir: $LOG_DIR'
      echo '===================================='
      echo

      if [ ! -f '$DIR/$script' ]; then
        echo '[ERROR] script not found: $DIR/$script'
        echo 127 > '$statusfile'
        echo
        echo 'Press Enter to close this terminal...'
        read -r
        exit 127
      fi

      if [ ! -x '$DIR/$script' ]; then
        echo '[ERROR] script not executable: $DIR/$script'
        echo 'Run: chmod +x $DIR/$script'
        echo 126 > '$statusfile'
        echo
        echo 'Press Enter to close this terminal...'
        read -r
        exit 126
      fi

      '$DIR/$script'
      rc=\$?

      echo
      echo '========== TERMINAL EXIT =========='
      echo 'time: '\"\$(date -Iseconds)\"
      echo 'title: $title'
      echo 'exit_code:' \$rc
      echo 'logfile: $logfile'
      echo '==================================='
      echo \$rc > '$statusfile'

      echo
      echo 'Press Enter to close this terminal...'
      read -r

      exit \$rc
    } 2>&1 | tee -a '$logfile'
  " >> "$LOG_DIR/gnome_terminal_launch.log" 2>&1 &
}

start_one "TF_STATIC1" "lidar_to_base_tf.sh"
start_one "TF_STATIC2" "map_to_init_tf.sh"
start_one "TF_STATIC3" "lidar_to_base_highrate_tf.sh"
start_one "LIVOX" "livox.sh"
start_one "FASTLIO" "fastlio.sh"
start_one "HIGHRATE_BODY" "highrate_body.sh"
start_one "world_tf" "world_tf.sh"
start_one "TF_ROBOT_OFFSET" "robot_tf_offsets.sh"
start_one "TF_LIDAR_OFFSET" "lidar_tf_offset.sh"
start_one "CAN_SERIAL" "serial_driver.sh"
start_one "LIDAR_XYZR" "lidar_xyzr.sh"

should_start_plotjuggler() {
  case "${RC_AUTOSTART_PLOTJUGGLER:-auto}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    0|false|FALSE|no|NO|off|OFF)
      return 1
      ;;
    auto)
      [ "$RUN_REASON" != "can_restart" ]
      return
      ;;
    *)
      echo "[WARN] Unknown RC_AUTOSTART_PLOTJUGGLER=${RC_AUTOSTART_PLOTJUGGLER}; using auto." | tee -a "$LOG_DIR/start_summary.txt"
      [ "$RUN_REASON" != "can_restart" ]
      return
      ;;
  esac
}

if should_start_plotjuggler; then
  start_one "PLOTJUGGLER" "plotjuggler_tf_compare.sh"
else
  echo "[$(date -Iseconds)] skipped PLOTJUGGLER for run_reason=$RUN_REASON" | tee -a "$LOG_DIR/start_summary.txt"
fi

if [ -n "$(jobs -p)" ]; then
  disown
fi

{
  echo "[$(date -Iseconds)] Started all terminals."
  echo "Log dir: $LOG_DIR"
  echo "Latest: $LOG_ROOT/latest"
} | tee -a "$LOG_DIR/start_summary.txt"
