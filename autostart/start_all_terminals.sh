#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${RC_LIDAR_ROOT:-$(cd "$DIR/.." && pwd)}"
WORKSPACE_DIR="${RC_LIDAR_WORKSPACE:-$REPO_ROOT}"
LOG_ROOT="${RC_LIDAR_LOG_ROOT:-$WORKSPACE_DIR/logs/autostart}"

# manual：手动启动
# boot：开机启动
# can_restart：CAN 报文触发重启
RUN_REASON="${RUN_REASON:-manual}"
RC_LIDAR_MODE="${RC_LIDAR_MODE:-tracking}"
if [ "${RC_AUTOSTART_READY_GATES+x}" != "x" ]; then
  RC_AUTOSTART_READY_GATES=0
fi
case "$RC_LIDAR_MODE" in
  tracking|mapping|relocalization)
    ;;
  *)
    echo "[ERROR] Unknown RC_LIDAR_MODE=$RC_LIDAR_MODE. Use tracking, mapping, or relocalization." >&2
    exit 2
    ;;
esac
if [ "${RC_LOCALIZATION_MAP_PATH+x}" = "x" ] && [ -n "${RC_LOCALIZATION_MAP_PATH:-}" ]; then
  RC_LIDAR_MAP_PATH="$RC_LOCALIZATION_MAP_PATH"
else
  RC_LIDAR_MAP_PATH="${RC_LIDAR_MAP_PATH:-$WORKSPACE_DIR/maps/mid360_map.pcd}"
fi
RC_LOCALIZATION_MAP_PATH="$RC_LIDAR_MAP_PATH"
RC_RAW_POSE_TOPIC="${RC_RAW_POSE_TOPIC:-/robot/pose}"
RC_FILTERED_POSE_TOPIC="${RC_FILTERED_POSE_TOPIC:-/robot/pose_filtered}"
RC_HEALTH_MONITOR="${RC_HEALTH_MONITOR:-${RC_THERMAL_POSE_MONITOR:-1}}"
RC_POSE_FILTER="${RC_POSE_FILTER:-0}"
RC_POSE_FILTER_SELECTED="${RC_POSE_FILTER_SELECTED:-normal}"
RC_POSE_FILTER_BETA="${RC_POSE_FILTER_BETA:-0}"
RC_BETA_FILTERED_POSE_TOPIC="${RC_BETA_FILTERED_POSE_TOPIC:-/robot/pose_filtered_beta}"
RC_POSE_FILTER_STABLE="${RC_POSE_FILTER_STABLE:-0}"
RC_STABLE_FILTERED_POSE_TOPIC="${RC_STABLE_FILTERED_POSE_TOPIC:-/robot/pose_filtered_stable}"
RC_CONTINUOUS_POSE="${RC_CONTINUOUS_POSE:-auto}"
RC_CONTINUOUS_POSE_OUTPUT_TOPIC="${RC_CONTINUOUS_POSE_OUTPUT_TOPIC:-/robot/pose_continuous}"
RC_CONTINUOUS_POSE_STAMP_MATCH_MODE="${RC_CONTINUOUS_POSE_STAMP_MATCH_MODE:-stamp}"
RC_CONTINUOUS_POSE_MAX_STAMP_DELTA="${RC_CONTINUOUS_POSE_MAX_STAMP_DELTA:-0.03}"
RC_AUTOSTART_DRY_RUN="${RC_AUTOSTART_DRY_RUN:-0}"
RC_AUTOSTART_CANFD="${RC_AUTOSTART_CANFD:-0}"
RC_CAN_RESTART_LISTENER="${RC_CAN_RESTART_LISTENER:-auto}"
RC_CANFD_RESTART_ENABLED="${RC_CANFD_RESTART_ENABLED:-1}"
RC_CAN_DEBUG_CSV="${RC_CAN_DEBUG_CSV:-0}"
RC_CAN_DEBUG_CSV_PATH="${RC_CAN_DEBUG_CSV_PATH:-}"
RC_CAN_DEBUG_LOG_PERIOD_SEC="${RC_CAN_DEBUG_LOG_PERIOD_SEC:-1.0}"
RC_READY_TOPIC_SPIN_TIME="${RC_READY_TOPIC_SPIN_TIME:-2}"
RC_READY_SAMPLE_TIMEOUT_SEC="${RC_READY_SAMPLE_TIMEOUT_SEC:-4}"
RC_READY_LIVOX_TIMEOUT_SEC="${RC_READY_LIVOX_TIMEOUT_SEC:-30}"
RC_READY_FASTLIO_TIMEOUT_SEC="${RC_READY_FASTLIO_TIMEOUT_SEC:-75}"
RC_READY_MAP_TIMEOUT_SEC="${RC_READY_MAP_TIMEOUT_SEC:-90}"
RC_READY_POSE_TIMEOUT_SEC="${RC_READY_POSE_TIMEOUT_SEC:-45}"
RC_READY_FILTER_TIMEOUT_SEC="${RC_READY_FILTER_TIMEOUT_SEC:-45}"
RC_READY_TF_COMPARE_TIMEOUT_SEC="${RC_READY_TF_COMPARE_TIMEOUT_SEC:-30}"
RC_READY_PLOTJUGGLER_TIMEOUT_SEC="${RC_READY_PLOTJUGGLER_TIMEOUT_SEC:-20}"
RC_READY_LIVOX_LIDAR_TOPIC="${RC_READY_LIVOX_LIDAR_TOPIC:-/livox/lidar}"
RC_READY_LIVOX_IMU_TOPIC="${RC_READY_LIVOX_IMU_TOPIC:-/livox/imu}"
RC_READY_FASTLIO_ODOM_TOPIC="${RC_READY_FASTLIO_ODOM_TOPIC:-/Odometry}"
RC_READY_FASTLIO_CLOUD_TOPIC="${RC_READY_FASTLIO_CLOUD_TOPIC:-/cloud_registered}"
RC_READY_MAP_ODOM_TOPIC="${RC_READY_MAP_ODOM_TOPIC:-/map_lidar/odom}"
RC_READY_MAP_QUALITY_TOPIC="${RC_READY_MAP_QUALITY_TOPIC:-/map_lidar/quality}"
case "$RC_POSE_FILTER" in
  1|true|TRUE|yes|YES|on|ON)
    RC_POSE_FILTER_ENABLED=1
    RC_POSE_FILTER=1
    ;;
  0|false|FALSE|no|NO|off|OFF|"")
    RC_POSE_FILTER_ENABLED=0
    RC_POSE_FILTER=0
    ;;
  *)
    echo "[ERROR] Unknown RC_POSE_FILTER=$RC_POSE_FILTER. Use 0 or 1." >&2
    exit 2
    ;;
esac
case "$RC_POSE_FILTER_SELECTED" in
  normal|beta|stable)
    ;;
  *)
    echo "[ERROR] Unknown RC_POSE_FILTER_SELECTED=$RC_POSE_FILTER_SELECTED. Use normal, beta, or stable." >&2
    exit 2
    ;;
esac
case "$RC_POSE_FILTER_BETA" in
  1|true|TRUE|yes|YES|on|ON)
    RC_POSE_FILTER_BETA_ENABLED=1
    ;;
  0|false|FALSE|no|NO|off|OFF|"")
    RC_POSE_FILTER_BETA_ENABLED=0
    ;;
  *)
    echo "[ERROR] Unknown RC_POSE_FILTER_BETA=$RC_POSE_FILTER_BETA. Use 0 or 1." >&2
    exit 2
    ;;
esac
case "$RC_POSE_FILTER_STABLE" in
  1|true|TRUE|yes|YES|on|ON)
    RC_POSE_FILTER_STABLE_ENABLED=1
    ;;
  0|false|FALSE|no|NO|off|OFF|"")
    RC_POSE_FILTER_STABLE_ENABLED=0
    ;;
  *)
    echo "[ERROR] Unknown RC_POSE_FILTER_STABLE=$RC_POSE_FILTER_STABLE. Use 0 or 1." >&2
    exit 2
    ;;
esac
if [ "$RC_POSE_FILTER_ENABLED" = "1" ]; then
  if [ "$RC_POSE_FILTER_SELECTED" = "beta" ] && [ "$RC_POSE_FILTER_BETA_ENABLED" != "1" ]; then
    echo "[ERROR] RC_POSE_FILTER_SELECTED=beta requires RC_POSE_FILTER_BETA=1." >&2
    exit 2
  fi
  if [ "$RC_POSE_FILTER_SELECTED" = "stable" ] && [ "$RC_POSE_FILTER_STABLE_ENABLED" != "1" ]; then
    echo "[ERROR] RC_POSE_FILTER_SELECTED=stable requires RC_POSE_FILTER_STABLE=1." >&2
    exit 2
  fi
fi
if [ "${RC_CAN_POSE_TOPIC+x}" != "x" ] || [ -z "${RC_CAN_POSE_TOPIC:-}" ]; then
  if [ "$RC_LIDAR_MODE" = "relocalization" ] && {
    [ "$RC_CONTINUOUS_POSE" = "auto" ] ||
      [ "$RC_CONTINUOUS_POSE" = "1" ] ||
      [ "$RC_CONTINUOUS_POSE" = "true" ] ||
      [ "$RC_CONTINUOUS_POSE" = "TRUE" ] ||
      [ "$RC_CONTINUOUS_POSE" = "yes" ] ||
      [ "$RC_CONTINUOUS_POSE" = "YES" ] ||
      [ "$RC_CONTINUOUS_POSE" = "on" ] ||
      [ "$RC_CONTINUOUS_POSE" = "ON" ]
  }; then
    RC_CAN_POSE_TOPIC="$RC_CONTINUOUS_POSE_OUTPUT_TOPIC"
  elif [ "$RC_POSE_FILTER_ENABLED" = "0" ]; then
    RC_CAN_POSE_TOPIC="$RC_RAW_POSE_TOPIC"
  elif [ "$RC_POSE_FILTER_SELECTED" = "beta" ]; then
    RC_CAN_POSE_TOPIC="$RC_BETA_FILTERED_POSE_TOPIC"
  elif [ "$RC_POSE_FILTER_SELECTED" = "stable" ]; then
    RC_CAN_POSE_TOPIC="$RC_STABLE_FILTERED_POSE_TOPIC"
  else
    RC_CAN_POSE_TOPIC="$RC_FILTERED_POSE_TOPIC"
  fi
fi
RC_READY_RAW_POSE_TOPIC="${RC_READY_RAW_POSE_TOPIC:-$RC_RAW_POSE_TOPIC}"
RC_READY_FILTERED_POSE_TOPIC="${RC_READY_FILTERED_POSE_TOPIC:-$RC_CAN_POSE_TOPIC}"
BOOT_ID="$(cat /proc/sys/kernel/random/boot_id 2>/dev/null || echo unknown)"
BOOT_ID_SHORT="$(printf '%s' "$BOOT_ID" | cut -c1-8)"

current_uptime_sec() {
  awk '{printf "%.3f", $1}' /proc/uptime 2>/dev/null || printf "0.000"
}

START_UPTIME_SEC="$(current_uptime_sec)"
RUN_ID="$(date +%Y%m%d_%H%M%S)_${RUN_REASON}_${BOOT_ID_SHORT}_$$"
LOG_DIR="$LOG_ROOT/run_$RUN_ID"

sync_file() {
  local path="$1"
  sync -f "$path" >/dev/null 2>&1 || sync >/dev/null 2>&1 || true
}

elapsed_since_start() {
  local now
  now="$(current_uptime_sec)"
  awk -v now="$now" -v start="$START_UPTIME_SEC" 'BEGIN { printf "%.3f", now - start }'
}

summary_event() {
  local message="$1"
  echo "[$(date -Iseconds) uptime_sec=$(current_uptime_sec) elapsed_sec=$(elapsed_since_start)] $message" | tee -a "$LOG_DIR/start_summary.txt"
  sync_file "$LOG_DIR/start_summary.txt"
}

mkdir -p "$LOG_DIR"
if [ "$RC_AUTOSTART_DRY_RUN" = "1" ]; then
  ln -sfn "$LOG_DIR" "$LOG_ROOT/latest_dry_run"
else
  ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"
fi

export RUN_ID
export LOG_DIR
export RUN_REASON
export RC_LIDAR_MODE
export RC_LIDAR_ROOT="$REPO_ROOT"
export RC_LIDAR_WORKSPACE="$WORKSPACE_DIR"
export RC_LIDAR_AUTOSTART="$DIR"
export RC_LIDAR_MAP_PATH
export RC_LOCALIZATION_MAP_PATH
export RC_RAW_POSE_TOPIC
export RC_FILTERED_POSE_TOPIC
export RC_HEALTH_MONITOR
export RC_POSE_FILTER
export RC_POSE_FILTER_SELECTED
export RC_POSE_FILTER_BETA
export RC_BETA_FILTERED_POSE_TOPIC
export RC_POSE_FILTER_STABLE
export RC_STABLE_FILTERED_POSE_TOPIC
export RC_CONTINUOUS_POSE
export RC_CONTINUOUS_POSE_OUTPUT_TOPIC
export RC_CONTINUOUS_POSE_STAMP_MATCH_MODE
export RC_CONTINUOUS_POSE_MAX_STAMP_DELTA
export RC_AUTOSTART_CANFD
export RC_CAN_RESTART_LISTENER
export RC_CANFD_RESTART_ENABLED
export RC_CAN_POSE_TOPIC
export RC_CAN_DEBUG_CSV
export RC_CAN_DEBUG_CSV_PATH
export RC_CAN_DEBUG_LOG_PERIOD_SEC
export RC_AUTOSTART_READY_GATES
export RC_READY_TOPIC_SPIN_TIME
export RC_READY_SAMPLE_TIMEOUT_SEC
export RC_READY_LIVOX_TIMEOUT_SEC
export RC_READY_FASTLIO_TIMEOUT_SEC
export RC_READY_MAP_TIMEOUT_SEC
export RC_READY_POSE_TIMEOUT_SEC
export RC_READY_FILTER_TIMEOUT_SEC
export RC_READY_TF_COMPARE_TIMEOUT_SEC
export RC_READY_PLOTJUGGLER_TIMEOUT_SEC
export RC_READY_LIVOX_LIDAR_TOPIC
export RC_READY_LIVOX_IMU_TOPIC
export RC_READY_FASTLIO_ODOM_TOPIC
export RC_READY_FASTLIO_CLOUD_TOPIC
export RC_READY_MAP_ODOM_TOPIC
export RC_READY_MAP_QUALITY_TOPIC
export RC_READY_RAW_POSE_TOPIC
export RC_READY_FILTERED_POSE_TOPIC
export RC_LIDAR_BOOT_ID="${RC_LIDAR_BOOT_ID:-$BOOT_ID}"

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
  echo "boot_id: $BOOT_ID"
  echo "boot_id_short: $BOOT_ID_SHORT"
  echo "uptime_sec: $START_UPTIME_SEC"
  echo "epoch_sec: $(date +%s)"
  echo "pid: $$"
  echo "repo_root: $REPO_ROOT"
  echo "workspace: $WORKSPACE_DIR"
  echo "runtime_mode: $RC_LIDAR_MODE"
  echo "map_path: $RC_LIDAR_MAP_PATH"
  echo "localization_map: $RC_LOCALIZATION_MAP_PATH"
  echo "health_monitor: $RC_HEALTH_MONITOR"
  echo "raw_pose_topic: $RC_RAW_POSE_TOPIC"
  echo "filtered_pose_topic: $RC_FILTERED_POSE_TOPIC"
  echo "pose_filter: $RC_POSE_FILTER"
  echo "pose_filter_selected: $RC_POSE_FILTER_SELECTED"
  echo "pose_filter_beta: $RC_POSE_FILTER_BETA"
  echo "beta_filtered_pose_topic: $RC_BETA_FILTERED_POSE_TOPIC"
  echo "pose_filter_stable: $RC_POSE_FILTER_STABLE"
  echo "stable_filtered_pose_topic: $RC_STABLE_FILTERED_POSE_TOPIC"
  echo "can_pose_topic: $RC_CAN_POSE_TOPIC"
  echo "canfd: $RC_AUTOSTART_CANFD"
  echo "can_restart_listener: $RC_CAN_RESTART_LISTENER"
  echo "canfd_restart_enabled: $RC_CANFD_RESTART_ENABLED"
  echo "can_debug_csv: $RC_CAN_DEBUG_CSV"
  echo "can_debug_csv_path: ${RC_CAN_DEBUG_CSV_PATH:-}"
  echo "continuous_pose: $RC_CONTINUOUS_POSE"
  echo "continuous_stamp_match_mode: $RC_CONTINUOUS_POSE_STAMP_MATCH_MODE"
  echo "continuous_max_stamp_delta_sec: $RC_CONTINUOUS_POSE_MAX_STAMP_DELTA"
  echo "ready_gates: $RC_AUTOSTART_READY_GATES"
  echo "ready_gates_note: default off; set RC_AUTOSTART_READY_GATES=1 for deep staged startup diagnostics"
  echo "ready_topic_spin_time: $RC_READY_TOPIC_SPIN_TIME"
  echo "ready_sample_timeout_sec: $RC_READY_SAMPLE_TIMEOUT_SEC"
  echo "log_dir: $LOG_DIR"
  echo "dir: $DIR"
  echo "user: $(whoami)"
  echo "host: $(hostname)"
  echo "DISPLAY: ${DISPLAY:-}"
  echo "XDG_SESSION_TYPE: ${XDG_SESSION_TYPE:-}"
  echo "ROS_LOG_DIR: $ROS_LOG_DIR"
  echo "========================================="
} | tee "$LOG_DIR/start_summary.txt"
sync_file "$LOG_DIR/start_summary.txt"

if [ -x "$DIR/cleanup_autostart_logs.sh" ]; then
  if [ "$RC_AUTOSTART_DRY_RUN" = "1" ]; then
    RC_AUTOSTART_LOG_CLEANUP_DRY_RUN=1 "$DIR/cleanup_autostart_logs.sh" "$LOG_ROOT" > "$LOG_DIR/log_cleanup.log" 2>&1 || true
  elif [ "${RC_AUTOSTART_LOG_CLEANUP:-1}" = "1" ]; then
    "$DIR/cleanup_autostart_logs.sh" "$LOG_ROOT" > "$LOG_DIR/log_cleanup.log" 2>&1 || true
  else
    echo "log cleanup disabled by RC_AUTOSTART_LOG_CLEANUP=0" > "$LOG_DIR/log_cleanup.log"
  fi
else
  echo "cleanup_autostart_logs.sh not found or not executable, skipped." > "$LOG_DIR/log_cleanup.log"
fi
sync_file "$LOG_DIR/log_cleanup.log"

# 如果是 restart_lidar_stack.sh 调起来的，记录来源
if [ -n "${RESTART_LOG:-}" ]; then
  {
    echo "restart_log: $RESTART_LOG"
    echo "restart_time: $(date -Iseconds)"
    echo "run_reason: $RUN_REASON"
  } > "$LOG_DIR/restart_source.txt"
fi

if [ "$RC_AUTOSTART_DRY_RUN" = "1" ]; then
  summary_event "prestop: skipped for dry-run"
elif [ "${RC_AUTOSTART_SKIP_PRESTOP:-0}" = "1" ]; then
  summary_event "prestop: skipped by RC_AUTOSTART_SKIP_PRESTOP=1"
elif [ -x "$DIR/stop_all.sh" ]; then
  summary_event "prestop: running $DIR/stop_all.sh --quiet"
  "$DIR/stop_all.sh" --quiet > "$LOG_DIR/prestop.log" 2>&1 || true
  sync_file "$LOG_DIR/prestop.log"
else
  summary_event "prestop: stop_all.sh not found or not executable, skipped"
fi
sync_file "$LOG_DIR/start_summary.txt"

if [ -x "$DIR/canable2_preflight.sh" ]; then
  "$DIR/canable2_preflight.sh" --check-only > "$LOG_DIR/canable2_preflight.txt" 2>&1 || true
else
  echo "canable2_preflight.sh not found or not executable, skipped." > "$LOG_DIR/canable2_preflight.txt"
fi
sync_file "$LOG_DIR/canable2_preflight.txt"

resolve_headless_mode() {
  case "${RC_AUTOSTART_HEADLESS:-auto}" in
    1|true|TRUE|yes|YES|on|ON)
      echo 1
      ;;
    0|false|FALSE|no|NO|off|OFF)
      echo 0
      ;;
    auto)
      echo 1
      ;;
    *)
      echo "[WARN] Unknown RC_AUTOSTART_HEADLESS=${RC_AUTOSTART_HEADLESS}; using auto." | tee -a "$LOG_DIR/start_summary.txt" >&2
      echo 1
      ;;
  esac
}

RUN_HEADLESS="$(resolve_headless_mode)"
echo "headless: $RUN_HEADLESS" >> "$LOG_DIR/start_summary.txt"

if [ "$RC_LIDAR_MODE" = "relocalization" ] && [ ! -s "$RC_LIDAR_MAP_PATH" ]; then
  if [ "$RC_AUTOSTART_DRY_RUN" = "1" ]; then
    echo "[dry-run] relocalization map would be required: $RC_LIDAR_MAP_PATH" | tee -a "$LOG_DIR/start_summary.txt"
  else
    {
      echo "[ERROR] RC_LIDAR_MODE=relocalization needs a non-empty map file:"
      echo "        $RC_LIDAR_MAP_PATH"
      echo "Build one first with:"
      echo "        RC_LIDAR_MODE=mapping ./autostart/start_all_terminals.sh"
      echo "        ./autostart/save_fast_lio_map.sh"
    } | tee -a "$LOG_DIR/start_summary.txt"
    exit 3
  fi
fi

resolve_gui_terminal() {
  case "${RC_AUTOSTART_TERMINAL:-auto}" in
    auto)
      if command -v xfce4-terminal >/dev/null 2>&1; then
        echo "xfce4-terminal"
      elif command -v gnome-terminal >/dev/null 2>&1; then
        echo "gnome-terminal"
      else
        echo ""
      fi
      ;;
    xfce4-terminal|gnome-terminal)
      if command -v "${RC_AUTOSTART_TERMINAL}" >/dev/null 2>&1; then
        echo "${RC_AUTOSTART_TERMINAL}"
      else
        echo ""
      fi
      ;;
    *)
      echo "[WARN] Unknown RC_AUTOSTART_TERMINAL=${RC_AUTOSTART_TERMINAL}; using auto." | tee -a "$LOG_DIR/start_summary.txt" >&2
      RC_AUTOSTART_TERMINAL=auto resolve_gui_terminal
      ;;
  esac
}

GUI_TERMINAL=""
if [ "$RUN_HEADLESS" != "1" ]; then
  if [ -z "${DISPLAY:-}" ]; then
    echo "[ERROR] DISPLAY is empty. start_all_terminals.sh needs a graphical session." | tee -a "$LOG_DIR/start_summary.txt"
    exit 1
  fi

  GUI_TERMINAL="$(resolve_gui_terminal)"
  if [ -z "$GUI_TERMINAL" ]; then
    echo "[ERROR] No supported terminal found. Install xfce4-terminal or gnome-terminal, or run with RC_AUTOSTART_HEADLESS=1." | tee -a "$LOG_DIR/start_summary.txt"
    exit 1
  fi

  echo "gui_terminal: $GUI_TERMINAL" >> "$LOG_DIR/start_summary.txt"
fi

# 开机启动时桌面可能还没完全起来，手动启动多等几秒也没坏处
if [ "$RC_AUTOSTART_DRY_RUN" = "1" ]; then
  START_DELAY=0
else
  START_DELAY="${RC_AUTOSTART_START_DELAY:-3}"
fi
sleep "$START_DELAY"
summary_event "post_start_delay"

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
    echo "[$(date -Iseconds) uptime_sec=$(current_uptime_sec) elapsed_sec=$(elapsed_since_start)] launching $title -> $script"
    echo "logfile: $logfile"
    echo "pidfile: $pidfile"
  } >> "$LOG_DIR/start_summary.txt"

  if [ "$RC_AUTOSTART_DRY_RUN" = "1" ]; then
    echo "[dry-run] would launch $title using $DIR/$script" | tee -a "$LOG_DIR/start_summary.txt"
    return 0
  fi

  if [ "$RUN_HEADLESS" = "1" ]; then
    setsid bash -lc "
      echo \$\$ > '$pidfile'

      export RUN_ID='$RUN_ID'
      export LOG_DIR='$LOG_DIR'
      export RUN_REASON='$RUN_REASON'
      export ROS_LOG_DIR='$ROS_LOG_DIR'
      export RCUTILS_LOGGING_USE_STDOUT=1
      export RCUTILS_LOGGING_BUFFERED_STREAM=0

      {
        echo '========== BACKGROUND START =========='
        echo 'time: '\"\$(date -Iseconds)\"
        echo 'title: $title'
        echo 'script: $DIR/$script'
        echo 'pidfile: $pidfile'
        echo 'logfile: $logfile'
        echo 'log_dir: $LOG_DIR'
        echo '======================================'
        echo

        if [ ! -f '$DIR/$script' ]; then
          echo '[ERROR] script not found: $DIR/$script'
          echo 127 > '$statusfile'
          exit 127
        fi

        if [ ! -x '$DIR/$script' ]; then
          echo '[ERROR] script not executable: $DIR/$script'
          echo 'Run: chmod +x $DIR/$script'
          echo 126 > '$statusfile'
          exit 126
        fi

        '$DIR/$script'
        rc=\$?

        echo
        echo '========== BACKGROUND EXIT =========='
        echo 'time: '\"\$(date -Iseconds)\"
        echo 'title: $title'
        echo 'exit_code:' \$rc
        echo 'logfile: $logfile'
        echo '====================================='
        echo \$rc > '$statusfile'

        exit \$rc
      } >> '$logfile' 2>&1
    " >> "$LOG_DIR/headless_launch.log" 2>&1 &
    return 0
  fi

  local runner="$LOG_DIR/${title}.terminal_runner.sh"
  cat > "$runner" <<EOF
#!/usr/bin/env bash
echo \$\$ > '$pidfile'

export RUN_ID='$RUN_ID'
export LOG_DIR='$LOG_DIR'
export RUN_REASON='$RUN_REASON'
export ROS_LOG_DIR='$ROS_LOG_DIR'
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0

{
  echo '========== TERMINAL START =========='
  echo 'time: '"\$(date -Iseconds)"
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
  echo 'time: '"\$(date -Iseconds)"
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
EOF
  chmod +x "$runner"

  if [ "$GUI_TERMINAL" = "xfce4-terminal" ]; then
    xfce4-terminal --disable-server --title="$title" --command "$runner" >> "$LOG_DIR/terminal_launch.log" 2>&1 &
  else
    gnome-terminal --title="$title" -- "$runner" >> "$LOG_DIR/terminal_launch.log" 2>&1 &
  fi

  sleep "${RC_AUTOSTART_TERMINAL_LAUNCH_DELAY:-0.4}"
}

gui_display_available() {
  case "${RC_AUTOSTART_ALLOW_HEADLESS_GUI:-0}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
  esac

  if [ -z "${DISPLAY:-}" ]; then
    return 1
  fi

  if command -v xdpyinfo >/dev/null 2>&1; then
    xdpyinfo >/dev/null 2>&1
    return
  fi

  return 0
}

should_start_plotjuggler() {
  case "${RC_AUTOSTART_PLOTJUGGLER:-auto}" in
    1|true|TRUE|yes|YES|on|ON)
      if gui_display_available; then
        return 0
      fi
      summary_event "skipped PLOTJUGGLER: RC_AUTOSTART_PLOTJUGGLER=1 but no reachable graphical display"
      return 1
      ;;
    0|false|FALSE|no|NO|off|OFF)
      return 1
      ;;
    auto)
      [ "$RUN_REASON" = "manual" ] && gui_display_available
      return
      ;;
    *)
      echo "[WARN] Unknown RC_AUTOSTART_PLOTJUGGLER=${RC_AUTOSTART_PLOTJUGGLER}; using auto." | tee -a "$LOG_DIR/start_summary.txt"
      [ "$RUN_REASON" = "manual" ] && gui_display_available
      return
      ;;
  esac
}

if should_start_plotjuggler; then
  START_PLOTJUGGLER=1
else
  START_PLOTJUGGLER=0
fi

should_start_tf_compare() {
  case "${RC_AUTOSTART_TF_COMPARE:-auto}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    0|false|FALSE|no|NO|off|OFF)
      return 1
      ;;
    auto)
      [ "$START_PLOTJUGGLER" = "1" ]
      return
      ;;
    *)
      echo "[WARN] Unknown RC_AUTOSTART_TF_COMPARE=${RC_AUTOSTART_TF_COMPARE}; using auto." | tee -a "$LOG_DIR/start_summary.txt"
      [ "$START_PLOTJUGGLER" = "1" ]
      return
      ;;
  esac
}

should_start_lidar_xyzr() {
  case "${RC_AUTOSTART_LIDAR_XYZR:-auto}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    0|false|FALSE|no|NO|off|OFF)
      return 1
      ;;
    auto)
      [ "$RUN_HEADLESS" != "1" ]
      return
      ;;
    *)
      echo "[WARN] Unknown RC_AUTOSTART_LIDAR_XYZR=${RC_AUTOSTART_LIDAR_XYZR}; using auto." | tee -a "$LOG_DIR/start_summary.txt"
      [ "$RUN_HEADLESS" != "1" ]
      return
      ;;
  esac
}

should_start_can_monitor() {
  case "${RC_AUTOSTART_CAN_MONITOR:-auto}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    0|false|FALSE|no|NO|off|OFF)
      return 1
      ;;
    auto)
      [ "$RUN_REASON" = "manual" ] && gui_display_available
      return
      ;;
    *)
      echo "[WARN] Unknown RC_AUTOSTART_CAN_MONITOR=${RC_AUTOSTART_CAN_MONITOR}; using auto." | tee -a "$LOG_DIR/start_summary.txt"
      [ "$RUN_REASON" = "manual" ] && gui_display_available
      return
      ;;
  esac
}

legacy_can_serial_enabled() {
  case "${RC_AUTOSTART_LEGACY_CAN_SERIAL:-0}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

canfd_enabled() {
  case "${RC_AUTOSTART_CANFD:-0}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

should_start_can_restart_listener() {
  if legacy_can_serial_enabled; then
    summary_event "skipped CAN_RESTART listener; legacy CAN_SERIAL already listens for restart frames"
    return 1
  fi

  case "${RC_CAN_RESTART_LISTENER:-auto}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    0|false|FALSE|no|NO|off|OFF)
      return 1
      ;;
    auto)
      if canfd_enabled; then
        summary_event "skipped CAN_RESTART listener; CANFD_POSE listens for FD CAN restart frames"
      fi
      return 1
      ;;
    *)
      echo "[WARN] Unknown RC_CAN_RESTART_LISTENER=${RC_CAN_RESTART_LISTENER}; using auto." | tee -a "$LOG_DIR/start_summary.txt"
      canfd_enabled
      return
      ;;
  esac
}

start_one_gui_terminal() {
  local title="$1"
  local script="$2"
  local pidfile="/tmp/rc_autostart_${title}.pid"
  local logfile="$LOG_DIR/${title}.log"
  local statusfile="$LOG_DIR/${title}.status"

  if [ "$RC_AUTOSTART_DRY_RUN" = "1" ]; then
    echo "[dry-run] would launch GUI terminal $title using $DIR/$script" | tee -a "$LOG_DIR/start_summary.txt"
    return 0
  fi

  if ! gui_display_available; then
    summary_event "no reachable graphical display for $title; launching as background log instead"
    start_one "$title" "$script"
    return 0
  fi

  local terminal
  terminal="$(resolve_gui_terminal)"
  if [ -z "$terminal" ]; then
    summary_event "no GUI terminal for $title; launching as background log instead"
    start_one "$title" "$script"
    return 0
  fi

  local runner="$LOG_DIR/${title}.terminal_runner.sh"
  cat > "$runner" <<EOF
#!/usr/bin/env bash
echo \$\$ > '$pidfile'

export RUN_ID='$RUN_ID'
export LOG_DIR='$LOG_DIR'
export RUN_REASON='$RUN_REASON'
export ROS_LOG_DIR='$ROS_LOG_DIR'
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0

{
  echo '========== TERMINAL START =========='
  echo 'time: '"\$(date -Iseconds)"
  echo 'title: $title'
  echo 'script: $DIR/$script'
  echo 'pidfile: $pidfile'
  echo 'logfile: $logfile'
  echo 'log_dir: $LOG_DIR'
  echo '===================================='
  echo

  '$DIR/$script'
  rc=\$?

  echo
  echo '========== TERMINAL EXIT =========='
  echo 'time: '"\$(date -Iseconds)"
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
EOF
  chmod +x "$runner"

  {
    echo "[$(date -Iseconds) uptime_sec=$(current_uptime_sec) elapsed_sec=$(elapsed_since_start)] launching GUI $title -> $script"
    echo "logfile: $logfile"
    echo "pidfile: $pidfile"
  } >> "$LOG_DIR/start_summary.txt"

  if [ "$terminal" = "xfce4-terminal" ]; then
    xfce4-terminal --disable-server --title="$title" --command "$runner" >> "$LOG_DIR/terminal_launch.log" 2>&1 &
  else
    gnome-terminal --title="$title" -- "$runner" >> "$LOG_DIR/terminal_launch.log" 2>&1 &
  fi

  sleep "${RC_AUTOSTART_TERMINAL_LAUNCH_DELAY:-0.4}"
}

READY_GATE_ROS_ENV_READY=0

ready_gate_enabled() {
  case "${RC_AUTOSTART_READY_GATES:-1}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    0|false|FALSE|no|NO|off|OFF)
      return 1
      ;;
    *)
      summary_event "ready_gate: unknown RC_AUTOSTART_READY_GATES=${RC_AUTOSTART_READY_GATES}; enabled"
      return 0
      ;;
  esac
}

ready_gate_log() {
  local message="$1"
  echo "[$(date -Iseconds) uptime_sec=$(current_uptime_sec) elapsed_sec=$(elapsed_since_start)] $message" >> "$LOG_DIR/ready_gates.log"
  sync_file "$LOG_DIR/ready_gates.log"
}

prepare_ready_gate_ros_env() {
  if [ "$READY_GATE_ROS_ENV_READY" = "1" ]; then
    return 0
  fi

  if [ ! -f /opt/ros/humble/setup.bash ]; then
    summary_event "ready_gate: /opt/ros/humble/setup.bash missing; skipping ROS readiness checks"
    return 1
  fi

  if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    summary_event "ready_gate: $WORKSPACE_DIR/install/setup.bash missing; skipping ROS readiness checks"
    return 1
  fi

  unset CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE
  unset CONDA_EXE CONDA_SHLVL _CE_CONDA _CE_M PYTHONHOME

  local restore_nounset=0
  case "$-" in
    *u*)
      restore_nounset=1
      set +u
      ;;
  esac

  # shellcheck disable=SC1091
  if ! source /opt/ros/humble/setup.bash; then
    if [ "$restore_nounset" = "1" ]; then
      set -u
    fi
    summary_event "ready_gate: failed to source /opt/ros/humble/setup.bash; skipping ROS readiness checks"
    return 1
  fi

  # shellcheck disable=SC1091
  if ! source "$WORKSPACE_DIR/install/setup.bash"; then
    if [ "$restore_nounset" = "1" ]; then
      set -u
    fi
    summary_event "ready_gate: failed to source $WORKSPACE_DIR/install/setup.bash; skipping ROS readiness checks"
    return 1
  fi

  if [ "$restore_nounset" = "1" ]; then
    set -u
  fi

  if ! command -v ros2 >/dev/null 2>&1; then
    summary_event "ready_gate: ros2 command not found after sourcing setup files; skipping ROS readiness checks"
    return 1
  fi

  READY_GATE_ROS_ENV_READY=1
  return 0
}

ready_gate_timeout_or_default() {
  local value="$1"
  local fallback="$2"

  case "$value" in
    ''|*[!0-9]*)
      echo "$fallback"
      ;;
    *)
      echo "$value"
      ;;
  esac
}

ros_topic_has_publisher() {
  local topic="$1"
  local info
  local publishers

  info="$(ros2 topic info --no-daemon --spin-time "$RC_READY_TOPIC_SPIN_TIME" "$topic" 2>/dev/null || true)"
  publishers="$(awk '/Publisher count:/ { print $3; found=1 } END { if (!found) print 0 }' <<< "$info")"

  [ "${publishers:-0}" -gt 0 ] 2>/dev/null
}

ros_topic_has_sample() {
  local topic="$1"
  local sample_timeout

  sample_timeout="${RC_READY_SAMPLE_TIMEOUT_SEC:-4}"
  case "$sample_timeout" in
    ''|*[!0-9.]*)
      sample_timeout=4
      ;;
  esac

  timeout "${sample_timeout}s" ros2 topic echo --once --no-daemon --spin-time "$RC_READY_TOPIC_SPIN_TIME" "$topic" >/dev/null 2>&1
}

wait_for_ready_topic() {
  local label="$1"
  local topic="$2"
  local timeout_sec
  local require_sample="${4:-1}"
  local deadline
  local reason="not checked"
  local last_reason=""

  timeout_sec="$(ready_gate_timeout_or_default "${3:-30}" 30)"

  if [ "$RC_AUTOSTART_DRY_RUN" = "1" ]; then
    summary_event "ready_gate: dry-run skip $label ($topic)"
    return 0
  fi

  if ! ready_gate_enabled; then
    return 0
  fi

  if ! prepare_ready_gate_ros_env; then
    ready_gate_log "skip $label ($topic): ROS environment unavailable"
    return 0
  fi

  summary_event "ready_gate: waiting for $label ($topic, timeout=${timeout_sec}s)"
  ready_gate_log "waiting for $label topic=$topic timeout=${timeout_sec}s require_sample=$require_sample"
  deadline=$((SECONDS + timeout_sec))

  while [ "$SECONDS" -lt "$deadline" ]; do
    if ros_topic_has_publisher "$topic"; then
      if [ "$require_sample" != "1" ]; then
        summary_event "ready_gate: $label ready ($topic publisher)"
        ready_gate_log "ready $label topic=$topic publisher=1 sample=not-required"
        return 0
      fi

      if ros_topic_has_sample "$topic"; then
        summary_event "ready_gate: $label ready ($topic sample)"
        ready_gate_log "ready $label topic=$topic publisher=1 sample=1"
        return 0
      fi

      reason="publisher present, no sample yet"
    else
      reason="no publisher"
    fi

    if [ "$reason" != "$last_reason" ]; then
      ready_gate_log "waiting $label topic=$topic reason=$reason"
      last_reason="$reason"
    fi

    sleep 1
  done

  summary_event "ready_gate: WARN timeout waiting for $label ($topic, last=$reason); continuing"
  ready_gate_log "timeout $label topic=$topic last=$reason"
  return 0
}

wait_for_plotjuggler_ready_topics() {
  if [ "$RC_LIDAR_MODE" = "relocalization" ]; then
    wait_for_ready_topic "PlotJuggler raw pose" "$RC_READY_RAW_POSE_TOPIC" "$RC_READY_PLOTJUGGLER_TIMEOUT_SEC" 1
    wait_for_ready_topic "PlotJuggler map quality" "$RC_READY_MAP_QUALITY_TOPIC" "$RC_READY_PLOTJUGGLER_TIMEOUT_SEC" 1

    if should_start_continuous_pose; then
      wait_for_ready_topic "PlotJuggler continuous pose" "${RC_CONTINUOUS_POSE_OUTPUT_TOPIC:-/robot/pose_continuous}" "$RC_READY_PLOTJUGGLER_TIMEOUT_SEC" 1
    fi

    if should_start_odom_only_compare; then
      wait_for_ready_topic "PlotJuggler odom-only pose" "${RC_ODOM_ONLY_POSE_TOPIC:-/robot/pose_odom_only}" "$RC_READY_PLOTJUGGLER_TIMEOUT_SEC" 1
    fi

    if [ "$RC_POSE_FILTER_ENABLED" = "1" ]; then
      wait_for_ready_topic "PlotJuggler filtered pose" "/robot/pose_filtered" "$RC_READY_PLOTJUGGLER_TIMEOUT_SEC" 1
      if [ "$RC_READY_FILTERED_POSE_TOPIC" != "/robot/pose_filtered" ]; then
        wait_for_ready_topic "PlotJuggler selected filtered pose" "$RC_READY_FILTERED_POSE_TOPIC" "$RC_READY_PLOTJUGGLER_TIMEOUT_SEC" 1
      fi
    fi
  fi

  if should_start_tf_compare; then
    wait_for_ready_topic "PlotJuggler TF robot base" "/tf_compare/robot_origin_to_robot_base" "$RC_READY_PLOTJUGGLER_TIMEOUT_SEC" 1
    wait_for_ready_topic "PlotJuggler TF high-rate robot base" "/tf_compare/robot_origin_to_robot_base_highrate" "$RC_READY_PLOTJUGGLER_TIMEOUT_SEC" 1
    wait_for_ready_topic "PlotJuggler TF map to body" "/tf_compare/map_to_body" "$RC_READY_PLOTJUGGLER_TIMEOUT_SEC" 1
  fi
}

should_start_thermal_pose_monitor() {
  case "${RC_HEALTH_MONITOR:-1}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    0|false|FALSE|no|NO|off|OFF|"")
      return 1
      ;;
    *)
      echo "[WARN] Unknown RC_HEALTH_MONITOR=${RC_HEALTH_MONITOR}; disabled." | tee -a "$LOG_DIR/start_summary.txt"
      return 1
      ;;
  esac
}

should_start_ready_notify() {
  case "${RC_READY_NOTIFY:-auto}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    0|false|FALSE|no|NO|off|OFF)
      return 1
      ;;
    auto)
      [ "$RC_LIDAR_MODE" = "relocalization" ] && [ "$RUN_REASON" != "can_restart" ]
      return
      ;;
    *)
      echo "[WARN] Unknown RC_READY_NOTIFY=${RC_READY_NOTIFY}; using auto." | tee -a "$LOG_DIR/start_summary.txt"
      [ "$RC_LIDAR_MODE" = "relocalization" ] && [ "$RUN_REASON" != "can_restart" ]
      return
      ;;
  esac
}

should_start_odom_only_compare() {
  case "${RC_ODOM_ONLY_COMPARE:-auto}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    0|false|FALSE|no|NO|off|OFF)
      return 1
      ;;
    auto)
      [ "$RC_LIDAR_MODE" = "relocalization" ] && [ "$START_PLOTJUGGLER" = "1" ]
      return
      ;;
    *)
      echo "[WARN] Unknown RC_ODOM_ONLY_COMPARE=${RC_ODOM_ONLY_COMPARE}; using auto." | tee -a "$LOG_DIR/start_summary.txt"
      [ "$RC_LIDAR_MODE" = "relocalization" ] && [ "$START_PLOTJUGGLER" = "1" ]
      return
      ;;
  esac
}

should_start_continuous_pose() {
  case "${RC_CONTINUOUS_POSE:-auto}" in
    1|true|TRUE|yes|YES|on|ON)
      [ "$RC_LIDAR_MODE" = "relocalization" ]
      return
      ;;
    0|false|FALSE|no|NO|off|OFF)
      return 1
      ;;
    auto)
      [ "$RC_LIDAR_MODE" = "relocalization" ]
      return
      ;;
    *)
      echo "[WARN] Unknown RC_CONTINUOUS_POSE=${RC_CONTINUOUS_POSE}; using auto." | tee -a "$LOG_DIR/start_summary.txt"
      [ "$RC_LIDAR_MODE" = "relocalization" ]
      return
      ;;
  esac
}

start_one "TF_STATIC1" "lidar_to_base_tf.sh"
start_one "TF_STATIC2" "map_to_init_tf.sh"
start_one "TF_STATIC3" "lidar_to_base_highrate_tf.sh"
if [ "$RC_LIDAR_MODE" = "relocalization" ]; then
  summary_event "skipped TF_MAP_CAMERA_INIT because MAP_LOCALIZER publishes map -> camera_init"
else
  start_one "TF_MAP_CAMERA_INIT" "map_to_camera_init_tf.sh"
fi
start_one "LIVOX" "livox.sh"
wait_for_ready_topic "Livox lidar" "$RC_READY_LIVOX_LIDAR_TOPIC" "$RC_READY_LIVOX_TIMEOUT_SEC" 1
wait_for_ready_topic "Livox imu" "$RC_READY_LIVOX_IMU_TOPIC" "$RC_READY_LIVOX_TIMEOUT_SEC" 1
start_one "FASTLIO" "fastlio.sh"
start_one "HIGHRATE_BODY" "highrate_body.sh"
wait_for_ready_topic "FastLIO odometry" "$RC_READY_FASTLIO_ODOM_TOPIC" "$RC_READY_FASTLIO_TIMEOUT_SEC" 1
wait_for_ready_topic "FastLIO registered cloud" "$RC_READY_FASTLIO_CLOUD_TOPIC" "$RC_READY_FASTLIO_TIMEOUT_SEC" 1
summary_event "skipped world_tf; production TF uses map -> robot_origin"
if [ "$RC_LIDAR_MODE" = "mapping" ]; then
  start_one "POSE_INTERFACE" "pose_interface_mapping.sh"
elif [ "$RC_LIDAR_MODE" = "relocalization" ]; then
  start_one "MAP_LOCALIZER" "map_localizer.sh"
  wait_for_ready_topic "map localizer odometry" "$RC_READY_MAP_ODOM_TOPIC" "$RC_READY_MAP_TIMEOUT_SEC" 1
  wait_for_ready_topic "map localizer quality" "$RC_READY_MAP_QUALITY_TOPIC" "$RC_READY_MAP_TIMEOUT_SEC" 1
  start_one "POSE_INTERFACE" "pose_interface_relocalization.sh"
  if should_start_continuous_pose; then
    start_one "POSE_CONTINUOUS" "continuous_robot_pose.sh"
    wait_for_ready_topic "robot continuous pose" "$RC_CONTINUOUS_POSE_OUTPUT_TOPIC" "$RC_READY_FILTER_TIMEOUT_SEC" 1
  else
    summary_event "skipped POSE_CONTINUOUS"
  fi
  if should_start_odom_only_compare; then
    start_one "POSE_ODOM_ONLY" "pose_odom_only_debug.sh"
  else
    summary_event "skipped POSE_ODOM_ONLY compare"
  fi
  wait_for_ready_topic "robot raw pose" "$RC_READY_RAW_POSE_TOPIC" "$RC_READY_POSE_TIMEOUT_SEC" 1
  if [ "$RC_POSE_FILTER_ENABLED" = "0" ]; then
    summary_event "skipped POSE_FILTER because RC_POSE_FILTER=0"
  else
    start_one "POSE_FILTER" "pose_stability_filter.sh"
    wait_for_ready_topic "robot filtered pose" "/robot/pose_filtered" "$RC_READY_FILTER_TIMEOUT_SEC" 1
    if [ "$RC_POSE_FILTER_BETA_ENABLED" = "1" ]; then
      start_one "POSE_FILTER_BETA" "pose_stability_filter_beta.sh"
      wait_for_ready_topic "robot beta filtered pose" "$RC_BETA_FILTERED_POSE_TOPIC" "$RC_READY_FILTER_TIMEOUT_SEC" 1
    else
      summary_event "skipped POSE_FILTER_BETA because RC_POSE_FILTER_BETA=0"
    fi
    if [ "$RC_POSE_FILTER_STABLE_ENABLED" = "1" ]; then
      start_one "POSE_FILTER_STABLE" "pose_stability_filter_stable.sh"
      wait_for_ready_topic "robot stable filtered pose" "$RC_STABLE_FILTERED_POSE_TOPIC" "$RC_READY_FILTER_TIMEOUT_SEC" 1
    else
      summary_event "skipped POSE_FILTER_STABLE because RC_POSE_FILTER_STABLE=0"
    fi
    if [ "$RC_READY_FILTERED_POSE_TOPIC" != "/robot/pose_filtered" ]; then
      wait_for_ready_topic "selected CAN/filtered pose" "$RC_READY_FILTERED_POSE_TOPIC" "$RC_READY_FILTER_TIMEOUT_SEC" 1
    fi
  fi
  if should_start_ready_notify; then
    start_one "READY_NOTIFY" "relocalization_ready_notify.sh"
  else
    summary_event "skipped READY_NOTIFY"
  fi
else
  summary_event "tracking mode: skipped MAP_LOCALIZER and POSE_INTERFACE"
fi
if should_start_thermal_pose_monitor; then
  start_one "HEALTH_MONITOR" "thermal_pose_monitor.sh"
else
  summary_event "skipped HEALTH_MONITOR because RC_HEALTH_MONITOR is not enabled"
fi
if should_start_tf_compare; then
  start_one "TF_ROBOT_OFFSET" "robot_tf_offsets.sh"
  start_one "TF_LIDAR_OFFSET" "lidar_tf_offset.sh"
  wait_for_ready_topic "TF compare robot base" "/tf_compare/robot_origin_to_robot_base" "$RC_READY_TF_COMPARE_TIMEOUT_SEC" 1
  wait_for_ready_topic "TF compare high-rate robot base" "/tf_compare/robot_origin_to_robot_base_highrate" "$RC_READY_TF_COMPARE_TIMEOUT_SEC" 1
  wait_for_ready_topic "TF compare map to body" "/tf_compare/map_to_body" "$RC_READY_TF_COMPARE_TIMEOUT_SEC" 1
else
  summary_event "skipped TF compare publishers"
fi
if legacy_can_serial_enabled; then
  start_one "CAN_SERIAL" "serial_driver.sh"
else
  summary_event "skipped CAN_SERIAL; legacy Classic CAN is disabled by default"
fi
if should_start_can_restart_listener; then
  start_one "CAN_RESTART" "can_restart_listener.sh"
else
  summary_event "skipped CAN_RESTART listener"
fi
case "$RC_AUTOSTART_CANFD" in
  1|true|TRUE|yes|YES|on|ON)
    start_one "CANFD_POSE" "canfd_pose_sender.sh"
    ;;
  *)
    summary_event "skipped CANFD_POSE because RC_AUTOSTART_CANFD=0"
    ;;
esac
if should_start_can_monitor; then
  start_one_gui_terminal "CAN_MONITOR" "can_pose_monitor.sh"
else
  summary_event "skipped CAN_MONITOR"
fi
if should_start_lidar_xyzr; then
  start_one "LIDAR_XYZR" "lidar_xyzr.sh"
else
  summary_event "skipped LIDAR_XYZR debug printer"
fi

if [ "$START_PLOTJUGGLER" = "1" ]; then
  wait_for_plotjuggler_ready_topics
  start_one "PLOTJUGGLER" "plotjuggler_tf_compare.sh"
else
  summary_event "skipped PLOTJUGGLER for run_reason=$RUN_REASON"
fi

if [ -n "$(jobs -p)" ]; then
  disown
fi

summary_event "Started all terminals."
{
  echo "Log dir: $LOG_DIR"
  if [ "$RC_AUTOSTART_DRY_RUN" = "1" ]; then
    echo "Latest dry-run: $LOG_ROOT/latest_dry_run"
  else
    echo "Latest: $LOG_ROOT/latest"
  fi
} | tee -a "$LOG_DIR/start_summary.txt"
sync_file "$LOG_DIR/start_summary.txt"
