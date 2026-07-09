#!/usr/bin/env bash
set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${RC_LIDAR_WORKSPACE:-$(cd "$DIR/.." && pwd)}"
LOG_ROOT="${RC_LIDAR_LOG_ROOT:-$WORKSPACE_DIR/logs/autostart}"
RESTART_LOG_DIR="$LOG_ROOT/restart"
mkdir -p "$RESTART_LOG_DIR"

LOCK="/tmp/rc_lidar_restart.lock"
LAST_RESTART_UPTIME_FILE="$RESTART_LOG_DIR/last_restart_uptime_sec"
RESTART_MIN_INTERVAL_SEC="${RC_CAN_RESTART_MIN_INTERVAL_SEC:-15}"
DRY_RUN="${RC_AUTOSTART_DRY_RUN:-0}"

exec 9>"$LOCK"
if ! flock -n 9; then
  SKIP_ID="$(date +%Y%m%d_%H%M%S)_busy"
  SKIP_LOG="$RESTART_LOG_DIR/restart_${SKIP_ID}.log"
  {
    echo "========== restart skipped at $(date '+%F %T') =========="
    echo "reason: another restart_lidar_stack.sh is already running"
    echo "lock: $LOCK"
  } >> "$SKIP_LOG" 2>&1
  ln -sfn "$SKIP_LOG" "$LOG_ROOT/latest_restart"
  exit 0
fi

current_uptime_sec_int() {
  awk '{printf "%d", $1}' /proc/uptime 2>/dev/null || date +%s
}

case "$RESTART_MIN_INTERVAL_SEC" in
  ''|*[!0-9]*)
    RESTART_MIN_INTERVAL_SEC=15
    ;;
esac

NOW_UPTIME_SEC="$(current_uptime_sec_int)"
if [ "$DRY_RUN" != "1" ]; then
  LAST_RESTART_UPTIME_SEC="$(cat "$LAST_RESTART_UPTIME_FILE" 2>/dev/null || true)"
  if [ "$RESTART_MIN_INTERVAL_SEC" -gt 0 ] &&
    [[ "$LAST_RESTART_UPTIME_SEC" =~ ^[0-9]+$ ]] &&
    [ $((NOW_UPTIME_SEC - LAST_RESTART_UPTIME_SEC)) -lt "$RESTART_MIN_INTERVAL_SEC" ]; then
    SKIP_ID="$(date +%Y%m%d_%H%M%S)_cooldown"
    SKIP_LOG="$RESTART_LOG_DIR/restart_${SKIP_ID}.log"
    {
      echo "========== restart skipped at $(date '+%F %T') =========="
      echo "reason: restart cooldown"
      echo "now_uptime_sec: $NOW_UPTIME_SEC"
      echo "last_restart_uptime_sec: $LAST_RESTART_UPTIME_SEC"
      echo "min_interval_sec: $RESTART_MIN_INTERVAL_SEC"
    } >> "$SKIP_LOG" 2>&1
    ln -sfn "$SKIP_LOG" "$LOG_ROOT/latest_restart"
    exit 0
  fi
  printf '%s\n' "$NOW_UPTIME_SEC" > "$LAST_RESTART_UPTIME_FILE"
fi

RESTART_ID="$(date +%Y%m%d_%H%M%S)_can"
LOG="$RESTART_LOG_DIR/restart_${RESTART_ID}.log"
ln -sfn "$LOG" "$LOG_ROOT/latest_restart"

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
    "CAN_RESTART"
    "CANFD_POSE"
    "CANFD_TX_MONITOR"
    "CAN+SERIAL"
    "CAN_MONITOR"
    "LIDAR_XYZR"
    "TF_STATIC1"
    "TF_STATIC2"
    "TF_STATIC3"
    "TF_MAP_CAMERA_INIT"
    "HIGHRATE_BODY"
    "world_tf"
    "TF_ROBOT_OFFSET"
    "TF_LIDAR_OFFSET"
    "PLOTJUGGLER"
    "MAP_LOCALIZER"
    "POSE_INTERFACE"
    "POSE_CONTINUOUS"
    "MATCH_ORIGIN"
    "POSE_FILTER"
    "POSE_FILTER_BETA"
    "POSE_FILTER_STABLE"
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

  if [ "$DRY_RUN" = "1" ]; then
    echo "[dry-run] restart_lidar_stack.sh skipped stop/start phases."
    cd "$WORKSPACE_DIR"
    DEFAULT_RELOCALIZATION_MAP="$WORKSPACE_DIR/maps/mid360_map_20260625_005422.pcd"
    if [ -f "$DIR/lidar_ui_config.sh" ]; then
      # shellcheck disable=SC1091
      source "$DIR/lidar_ui_config.sh"
      load_lidar_ui_config
      load_lidar_user_defaults
    fi
    if [ -n "${RC_UI_BOOT_MAP:-}" ]; then
      DRY_RUN_MAP_PATH="$RC_UI_BOOT_MAP"
    else
      DRY_RUN_MAP_PATH="${RC_LOCALIZATION_MAP_PATH:-${RC_LIDAR_MAP_PATH:-$DEFAULT_RELOCALIZATION_MAP}}"
    fi
    DRY_RUN_INITIAL_GUESS="${RC_RELOCALIZATION_INITIAL_GUESS:-0}"
    DRY_RUN_ASSUME_MAP_ORIGIN="${RC_RELOCALIZATION_ASSUME_MAP_ORIGIN:-1}"
    DRY_RUN_INITIAL_X="${RC_RELOCALIZATION_INITIAL_X:-0.0}"
    DRY_RUN_INITIAL_Y="${RC_RELOCALIZATION_INITIAL_Y:-0.0}"
    DRY_RUN_INITIAL_Z="${RC_RELOCALIZATION_INITIAL_Z:-0.0}"
    DRY_RUN_INITIAL_YAW="${RC_RELOCALIZATION_INITIAL_YAW:-0.0}"
    DRY_RUN_INITIAL_YAW_DEG="$(awk -v rad="$DRY_RUN_INITIAL_YAW" 'BEGIN { pi = atan2(0, -1); printf "%.6f", rad * 180.0 / pi }')"
    echo "[dry-run] would_use_map: $DRY_RUN_MAP_PATH"
    echo "[dry-run] would_use_initial_guess: $DRY_RUN_INITIAL_GUESS"
    echo "[dry-run] would_use_assume_map_origin: $DRY_RUN_ASSUME_MAP_ORIGIN"
    echo "[dry-run] would_use_initial_xyz: $DRY_RUN_INITIAL_X,$DRY_RUN_INITIAL_Y,$DRY_RUN_INITIAL_Z"
    echo "[dry-run] would_use_initial_yaw_rad: $DRY_RUN_INITIAL_YAW"
    echo "[dry-run] would_use_initial_yaw_deg: $DRY_RUN_INITIAL_YAW_DEG"
    echo "========== dry-run done at $(date '+%F %T') =========="
    exit 0
  fi

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
    "$DIR/can_restart_listener.sh"
    "$DIR/canfd_pose_sender.sh"
    "$DIR/can_pose_monitor.sh"
    "$DIR/lidar_xyzr.sh"
    "$DIR/lidar_to_base_tf.sh"
    "$DIR/map_to_init_tf.sh"
    "$DIR/map_to_camera_init_tf.sh"
    "$DIR/lidar_to_base_highrate_tf.sh"
    "$DIR/highrate_body.sh"
    "$DIR/world_tf.sh"
    "$DIR/robot_tf_offsets.sh"
    "$DIR/lidar_tf_offset.sh"
    "$DIR/plotjuggler_tf_compare.sh"
    "$DIR/map_localizer.sh"
    "$DIR/pose_interface_mapping.sh"
    "$DIR/pose_interface_relocalization.sh"
    "$DIR/continuous_robot_pose.sh"
    "$DIR/match_origin_pose.sh"
    "$DIR/pose_stability_filter.sh"
    "$DIR/pose_stability_filter_beta.sh"
    "$DIR/thermal_pose_monitor.sh"
    "$DIR/relocalization_ready_notify.sh"
    "ros2 launch rc_serial_driver serial_driver.launch.py"
    "ros2 run lidar_localization canfd_pose_sender"
    "ros2 launch lidar_map_localization"
    "ros2 launch lidar_localization"
    "map_localizer"
    "pose_from_odom"
    "pose_from_tf"
    "continuous_robot_pose"
    "match_origin_pose"
    "can_pose_monitor.py"
    "canfd_pose_sender"
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
  cd "$WORKSPACE_DIR"

  # Do not let the new LiDAR stack inherit the restart flock fd. If this fd is
  # inherited by long-running ROS nodes, later CAN restart requests can never
  # acquire the lock even though the restart script has already finished.
  exec 9>&-

  DEFAULT_RELOCALIZATION_MAP="$WORKSPACE_DIR/maps/mid360_map_20260625_005422.pcd"
  if [ -f "$DIR/lidar_ui_config.sh" ]; then
    # shellcheck disable=SC1091
    source "$DIR/lidar_ui_config.sh"
    load_lidar_ui_config
    load_lidar_user_defaults
  fi
  if [ -n "${RC_UI_BOOT_MAP:-}" ]; then
    RESTART_MAP_PATH="$RC_UI_BOOT_MAP"
  else
    RESTART_MAP_PATH="${RC_LOCALIZATION_MAP_PATH:-${RC_LIDAR_MAP_PATH:-$DEFAULT_RELOCALIZATION_MAP}}"
  fi
  RESTART_CONTINUOUS_TOPIC="${RC_CONTINUOUS_POSE_OUTPUT_TOPIC:-/robot/pose_continuous}"
  RESTART_MATCH_ORIGIN_ENABLE="${RC_MATCH_ORIGIN_ENABLE:-0}"
  RESTART_MATCH_ORIGIN_OUTPUT_TOPIC="${RC_MATCH_ORIGIN_OUTPUT_TOPIC:-/robot/pose_match}"
  RESTART_MATCH_ORIGIN_STATUS_TOPIC="${RC_MATCH_ORIGIN_STATUS_TOPIC:-/robot/match_origin/status}"
  RESTART_CAN_POSE_TOPIC="${RC_CAN_POSE_TOPIC:-}"
  case "$RESTART_MATCH_ORIGIN_ENABLE" in
    1|true|TRUE|yes|YES|on|ON)
      if [ -z "$RESTART_CAN_POSE_TOPIC" ] || [ "$RESTART_CAN_POSE_TOPIC" = "$RESTART_CONTINUOUS_TOPIC" ]; then
        RESTART_CAN_POSE_TOPIC="$RESTART_MATCH_ORIGIN_OUTPUT_TOPIC"
      fi
      RESTART_READY_NOTIFY_POSE_TOPIC="${RC_READY_NOTIFY_POSE_TOPIC:-$RESTART_MATCH_ORIGIN_OUTPUT_TOPIC}"
      ;;
    *)
      RESTART_CAN_POSE_TOPIC="${RESTART_CAN_POSE_TOPIC:-$RESTART_CONTINUOUS_TOPIC}"
      RESTART_READY_NOTIFY_POSE_TOPIC="${RC_READY_NOTIFY_POSE_TOPIC:-$RESTART_CONTINUOUS_TOPIC}"
      ;;
  esac

  RUN_REASON="can_restart" \
    RC_LIDAR_MODE="${RC_LIDAR_MODE:-relocalization}" \
    RC_LOCALIZATION_MAP_PATH="$RESTART_MAP_PATH" \
    RC_LIDAR_MAP_PATH="$RESTART_MAP_PATH" \
    RC_RELOCALIZATION_ASSUME_MAP_ORIGIN="${RC_RELOCALIZATION_ASSUME_MAP_ORIGIN:-1}" \
    RC_RELOCALIZATION_INITIAL_GUESS="${RC_RELOCALIZATION_INITIAL_GUESS:-0}" \
    RC_RELOCALIZATION_INITIAL_X="${RC_RELOCALIZATION_INITIAL_X:-0.0}" \
    RC_RELOCALIZATION_INITIAL_Y="${RC_RELOCALIZATION_INITIAL_Y:-0.0}" \
    RC_RELOCALIZATION_INITIAL_Z="${RC_RELOCALIZATION_INITIAL_Z:-0.0}" \
    RC_RELOCALIZATION_INITIAL_ROLL="${RC_RELOCALIZATION_INITIAL_ROLL:-0.0}" \
    RC_RELOCALIZATION_INITIAL_PITCH="${RC_RELOCALIZATION_INITIAL_PITCH:-0.0}" \
    RC_RELOCALIZATION_INITIAL_YAW="${RC_RELOCALIZATION_INITIAL_YAW:-0.0}" \
    RC_AUTOSTART_HEADLESS="${RC_AUTOSTART_HEADLESS:-1}" \
    RC_AUTOSTART_PLOTJUGGLER="${RC_AUTOSTART_PLOTJUGGLER:-1}" \
    RC_AUTOSTART_LEGACY_CAN_SERIAL="${RC_AUTOSTART_LEGACY_CAN_SERIAL:-0}" \
    RC_AUTOSTART_CANFD="${RC_AUTOSTART_CANFD:-1}" \
    RC_AUTOSTART_CANFD_TX_MONITOR="${RC_AUTOSTART_CANFD_TX_MONITOR:-1}" \
    RC_CAN_RESTART_LISTENER="${RC_CAN_RESTART_LISTENER:-auto}" \
    RC_CANFD_RESTART_ENABLED="${RC_CANFD_RESTART_ENABLED:-1}" \
    RC_CANFD_RESTART_CAN_ID="${RC_CANFD_RESTART_CAN_ID:-0x777}" \
    RC_CAN_RESTART_CAN_ID="${RC_CAN_RESTART_CAN_ID:-${RC_CANFD_RESTART_CAN_ID:-0x777}}" \
    RC_CANFD_RESTART_PAYLOAD_HEX="${RC_CANFD_RESTART_PAYLOAD_HEX:-A55A01FE00000000}" \
    RC_CANFD_SERIAL_PRESENCE_CHECK_PERIOD_SEC="${RC_CANFD_SERIAL_PRESENCE_CHECK_PERIOD_SEC:-0.5}" \
    RC_CAN_RESTART_DEBOUNCE_SEC="${RC_CAN_RESTART_DEBOUNCE_SEC:-8.0}" \
    RC_CAN_RESTART_MIN_INTERVAL_SEC="${RC_CAN_RESTART_MIN_INTERVAL_SEC:-15}" \
    RC_CAN_DEBUG_CSV="${RC_CAN_DEBUG_CSV:-1}" \
    RC_MAP_LOCALIZER_LOCALIZE_PERIOD_SEC="${RC_MAP_LOCALIZER_LOCALIZE_PERIOD_SEC:-0.2}" \
    RC_MAP_LOCALIZER_EXECUTOR_THREADS="${RC_MAP_LOCALIZER_EXECUTOR_THREADS:-3}" \
    RC_MAP_LOCALIZER_ODOM_BUFFER_DURATION_SEC="${RC_MAP_LOCALIZER_ODOM_BUFFER_DURATION_SEC:-2.0}" \
    RC_MAP_LOCALIZER_MAX_SCAN_ODOM_DELTA_SEC="${RC_MAP_LOCALIZER_MAX_SCAN_ODOM_DELTA_SEC:-0.05}" \
    RC_MAP_LOCALIZER_MAX_SCAN_AGE_SEC="${RC_MAP_LOCALIZER_MAX_SCAN_AGE_SEC:-0.5}" \
    RC_MAP_LOCALIZER_MAP_LEAF_SIZE="${RC_MAP_LOCALIZER_MAP_LEAF_SIZE:-0.10}" \
    RC_MAP_LOCALIZER_SCAN_LEAF_SIZE="${RC_MAP_LOCALIZER_SCAN_LEAF_SIZE:-0.10}" \
    RC_FASTLIO_FILTER_SIZE_SURF="${RC_FASTLIO_FILTER_SIZE_SURF:-0.15}" \
    RC_FASTLIO_FILTER_SIZE_MAP="${RC_FASTLIO_FILTER_SIZE_MAP:-0.25}" \
    RC_FASTLIO_POINT_FILTER_NUM="${RC_FASTLIO_POINT_FILTER_NUM:-3}" \
    RC_FASTLIO_RUNTIME_POS_LOG_ENABLE="${RC_FASTLIO_RUNTIME_POS_LOG_ENABLE:-0}" \
    RC_FASTLIO_DET_RANGE="${RC_FASTLIO_DET_RANGE:-100}" \
    FASTLIO_RVIZ="${FASTLIO_RVIZ:-false}" \
    RC_CONTINUOUS_POSE="${RC_CONTINUOUS_POSE:-auto}" \
    RC_CONTINUOUS_POSE_OUTPUT_TOPIC="$RESTART_CONTINUOUS_TOPIC" \
    RC_CONTINUOUS_POSE_STAMP_MATCH_MODE="${RC_CONTINUOUS_POSE_STAMP_MATCH_MODE:-stamp}" \
    RC_CONTINUOUS_POSE_MAX_STAMP_DELTA="${RC_CONTINUOUS_POSE_MAX_STAMP_DELTA:-0.03}" \
    RC_MATCH_ORIGIN_ENABLE="$RESTART_MATCH_ORIGIN_ENABLE" \
    RC_MATCH_ORIGIN_INPUT_TOPIC="${RC_MATCH_ORIGIN_INPUT_TOPIC:-$RESTART_CONTINUOUS_TOPIC}" \
    RC_MATCH_ORIGIN_INPUT_STATUS_TOPIC="${RC_MATCH_ORIGIN_INPUT_STATUS_TOPIC:-${RC_CONTINUOUS_POSE_STATUS_TOPIC:-/robot/pose_continuous/status}}" \
    RC_MATCH_ORIGIN_OUTPUT_TOPIC="$RESTART_MATCH_ORIGIN_OUTPUT_TOPIC" \
    RC_MATCH_ORIGIN_OUTPUT_ODOM_TOPIC="${RC_MATCH_ORIGIN_OUTPUT_ODOM_TOPIC:-/robot/odom_match}" \
    RC_MATCH_ORIGIN_STATUS_TOPIC="$RESTART_MATCH_ORIGIN_STATUS_TOPIC" \
    RC_MATCH_ORIGIN_MIN_LOCK_SEC="${RC_MATCH_ORIGIN_MIN_LOCK_SEC:-2.5}" \
    RC_MATCH_ORIGIN_MIN_SAMPLES="${RC_MATCH_ORIGIN_MIN_SAMPLES:-20}" \
    RC_MATCH_ORIGIN_MAX_XY_STD="${RC_MATCH_ORIGIN_MAX_XY_STD:-0.08}" \
    RC_MATCH_ORIGIN_MAX_YAW_STD_DEG="${RC_MATCH_ORIGIN_MAX_YAW_STD_DEG:-5.0}" \
    RC_MATCH_ORIGIN_USE_QUALITY_GATE="${RC_MATCH_ORIGIN_USE_QUALITY_GATE:-0}" \
    RC_MATCH_ORIGIN_FITNESS_WARN="${RC_MATCH_ORIGIN_FITNESS_WARN:-0.30}" \
    RC_POSE_FILTER="${RC_POSE_FILTER:-0}" \
    RC_POSE_FILTER_SELECTED="${RC_POSE_FILTER_SELECTED:-normal}" \
    RC_POSE_FILTER_BETA="${RC_POSE_FILTER_BETA:-0}" \
    RC_BETA_FILTERED_POSE_TOPIC="${RC_BETA_FILTERED_POSE_TOPIC:-/robot/pose_filtered_beta}" \
    RC_POSE_FILTER_STABLE="${RC_POSE_FILTER_STABLE:-0}" \
    RC_STABLE_FILTERED_POSE_TOPIC="${RC_STABLE_FILTERED_POSE_TOPIC:-/robot/pose_filtered_stable}" \
    RC_ROBOT_POSE_YAW_FUSION="${RC_ROBOT_POSE_YAW_FUSION:-0}" \
    RC_CAN_POSE_TOPIC="$RESTART_CAN_POSE_TOPIC" \
    RC_CAN_POSE_TIMEOUT_SEC="${RC_CAN_POSE_TIMEOUT_SEC:-0.5}" \
    RC_CAN_RAW_ODOM_POSE_TOPIC="${RC_CAN_RAW_ODOM_POSE_TOPIC:-/Odometry}" \
    RC_CAN_RAW_ODOM_POSE_TIMEOUT_SEC="${RC_CAN_RAW_ODOM_POSE_TIMEOUT_SEC:-0.5}" \
    RC_CAN_RAW_ODOM_POSE_STRICT="${RC_CAN_RAW_ODOM_POSE_STRICT:-1}" \
    RC_CAN_RAW_ODOM_POSE_YAW_OFFSET_RAD="${RC_CAN_RAW_ODOM_POSE_YAW_OFFSET_RAD:-0.0}" \
    RC_CAN_USE_YAW_POSE_TOPIC="${RC_CAN_USE_YAW_POSE_TOPIC:-0}" \
    RC_CAN_YAW_POSE_TOPIC="${RC_CAN_YAW_POSE_TOPIC:-/robot/pose}" \
    RC_CAN_YAW_POSE_TOPIC_STRICT="${RC_CAN_YAW_POSE_TOPIC_STRICT:-1}" \
    RC_CAN_USE_RAW_ODOM_YAW="${RC_CAN_USE_RAW_ODOM_YAW:-0}" \
    RC_READY_NOTIFY_POSE_TOPIC="$RESTART_READY_NOTIFY_POSE_TOPIC" \
    RC_HEALTH_MONITOR="${RC_HEALTH_MONITOR:-1}" \
    RESTART_LOG="$LOG" \
    nohup "$DIR/start_all_terminals.sh" >> "$LOG" 2>&1 &

  echo "[5/5] restart command launched."
  echo "========== done at $(date '+%F %T') =========="
} >> "$LOG" 2>&1
