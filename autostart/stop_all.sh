#!/usr/bin/env bash
set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$DIR/.." && pwd)"
QUIET=0
STOP_WAIT_SEC="${RC_STOP_ALL_WAIT_SEC:-2}"
STOP_FORCE_WAIT_SEC="${RC_STOP_ALL_FORCE_WAIT_SEC:-1}"

case "$STOP_WAIT_SEC" in
  ''|*[!0-9]*)
    STOP_WAIT_SEC=2
    ;;
esac

case "$STOP_FORCE_WAIT_SEC" in
  ''|*[!0-9]*)
    STOP_FORCE_WAIT_SEC=1
    ;;
esac

for arg in "$@"; do
  case "$arg" in
    --quiet)
      QUIET=1
      ;;
    --help|-h)
      echo "Usage: $0 [--quiet]"
      echo "Stop RC LIDAR autostart ROS processes and remove stale pid files."
      exit 0
      ;;
  esac
done

log() {
  if [ "$QUIET" != "1" ]; then
    echo "[stop_all] $*"
  fi
}

log_always() {
  echo "[stop_all] $*"
}

should_kill_cmd() {
  local cmd="$1"

  case "$cmd" in
    *"$WORKSPACE_DIR"*|*"$DIR"*)
      return 0
      ;;
    *"ros2 launch my_static_tf "*|*"ros2 launch fast_lio "*|*"ros2 launch livox_ros_driver2 "*|*"ros2 launch rc_serial_driver "*|*"ros2 launch lidar_map_localization "*|*"ros2 launch lidar_localization "*)
      return 0
      ;;
    *"ros2 run robot_localization "*|*"ros2 run lidar_localization canfd_pose_sender"*|*"ros2 run lidar_localization pose_filter"*|*"ros2 run lidar_localization health_monitor"*)
      return 0
      ;;
    *"fastlio_mapping"*|*"livox_ros_driver2_node"*|*"rc_serial_driver_node"*|*"ekf_node"*)
      return 0
      ;;
    *"lidar_xyzr.py"*|*"map_localizer"*|*"pose_from_odom"*|*"pose_from_tf"*|*"continuous_robot_pose"*|*"canfd_pose_sender"*|*"/lidar_localization/pose_filter"*|*"/lidar_localization/health_monitor"*)
      return 0
      ;;
    *"thermal_pose_monitor.sh"*|*"thermal_pose_monitor.py"*)
      return 0
      ;;
    *"relocalization_ready_notify.sh"*|*"relocalization_ready_notify.py"*)
      return 0
      ;;
    *"static_transform_publisher"*base_link*|*"static_transform_publisher"*robot_base*|*"static_transform_publisher"*body*|*"static_transform_publisher"*init*|*"static_transform_publisher"*robot_origin*|*"static_transform_publisher"*camera_init*)
      return 0
      ;;
    *"rviz2"*fast_lio*|*"plotjuggler"*"TF Compare"*)
      return 0
      ;;
  esac

  return 1
}

kill_pid_safely() {
  local pid="$1"
  local signal="${2:-TERM}"

  if ! [[ "$pid" =~ ^[0-9]+$ ]]; then
    return 0
  fi

  if [ "$pid" = "$$" ]; then
    return 0
  fi

  if ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi

  local cmd
  cmd="$(ps -p "$pid" -o args= 2>/dev/null || true)"
  if [ -z "$cmd" ] || ! should_kill_cmd "$cmd"; then
    log "skip pid=$pid cmd=$cmd"
    return 0
  fi

  log "kill -$signal pid=$pid cmd=$cmd"

  local self_pgid
  local target_pgid
  self_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d ' ')"
  target_pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"

  if [ -n "${target_pgid:-}" ] && [ "$target_pgid" = "$self_pgid" ]; then
    log "skip pid=$pid in current process group cmd=$cmd"
    return 0
  fi

  pkill "-$signal" -P "$pid" 2>/dev/null || true
  kill "-$signal" "$pid" 2>/dev/null || true

  if [ -n "${target_pgid:-}" ] && [ "$target_pgid" != "$self_pgid" ] && [ "$target_pgid" != "1" ]; then
    kill "-$signal" -- "-$target_pgid" 2>/dev/null || true
  fi
}

kill_matching_processes() {
  local signal="$1"
  shift

  local pattern
  for pattern in "$@"; do
    local pids
    pids="$(pgrep -f "$pattern" 2>/dev/null || true)"
    local pid
    for pid in $pids; do
      kill_pid_safely "$pid" "$signal"
    done
  done
}

list_matching_pids() {
  local pattern
  local pid
  local cmd
  local seen=" "
  local self_pgid
  local target_pgid

  self_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d ' ')"

  for pattern in "${patterns[@]}"; do
    while IFS= read -r pid; do
      if ! [[ "$pid" =~ ^[0-9]+$ ]]; then
        continue
      fi

      if [ "$pid" = "$$" ]; then
        continue
      fi

      case "$seen" in
        *" $pid "*)
          continue
          ;;
      esac

      if ! kill -0 "$pid" 2>/dev/null; then
        continue
      fi

      cmd="$(ps -p "$pid" -o args= 2>/dev/null || true)"
      if [ -z "$cmd" ] || ! should_kill_cmd "$cmd"; then
        continue
      fi

      target_pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
      if [ -n "${target_pgid:-}" ] && [ "$target_pgid" = "$self_pgid" ]; then
        continue
      fi

      seen="$seen$pid "
      echo "$pid"
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  done
}

wait_for_matching_processes_to_exit() {
  local timeout_sec="$1"
  local deadline=$((SECONDS + timeout_sec))
  local pids=()

  while true; do
    mapfile -t pids < <(list_matching_pids)
    if [ "${#pids[@]}" -eq 0 ]; then
      return 0
    fi

    if [ "$SECONDS" -ge "$deadline" ]; then
      return 1
    fi

    sleep 0.5
  done
}

log_remaining_processes() {
  local label="$1"
  local pids=()
  local pid
  local pgid
  local stat
  local etime
  local cmd

  mapfile -t pids < <(list_matching_pids)
  if [ "${#pids[@]}" -eq 0 ]; then
    return 1
  fi

  log_always "$label: ${#pids[@]} matching process(es) still running"
  for pid in "${pids[@]}"; do
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
    stat="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
    etime="$(ps -o etime= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
    cmd="$(ps -p "$pid" -o args= 2>/dev/null || true)"
    log_always "remaining pid=$pid pgid=${pgid:-?} stat=${stat:-?} etime=${etime:-?} cmd=$cmd"
  done

  return 0
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
    "PLOTJUGGLER"
    "ROTATION_CALIB"
    "MAP_LOCALIZER"
    "POSE_INTERFACE"
    "POSE_ODOM_ONLY"
    "POSE_CONTINUOUS"
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
    local title
    for title in "${titles[@]}"; do
      wmctrl -c "$title" 2>/dev/null || true
    done
  fi

  if command -v xdotool >/dev/null 2>&1; then
    local title
    for title in "${titles[@]}"; do
      xdotool search --name "$title" windowkill 2>/dev/null || true
    done
  fi
}

patterns=(
  "$DIR/livox.sh"
  "$DIR/fastlio.sh"
  "$DIR/serial_driver.sh"
  "$DIR/can_restart_listener.sh"
  "$DIR/canfd_pose_sender.sh"
  "$DIR/canfd_tx_monitor.sh"
  "$DIR/can_pose_monitor.sh"
  "$DIR/lidar_xyzr.sh"
  "$DIR/lidar_to_base_tf.sh"
  "$DIR/map_to_init_tf.sh"
  "$DIR/map_to_camera_init_tf.sh"
  "$DIR/lidar_to_base_highrate_tf.sh"
  "$DIR/highrate_body.sh"
  "$DIR/plotjuggler_tf_compare.sh"
  "$DIR/map_localizer.sh"
  "$DIR/pose_interface_mapping.sh"
  "$DIR/pose_interface_relocalization.sh"
  "$DIR/pose_odom_only_debug.sh"
  "$DIR/continuous_robot_pose.sh"
  "$DIR/pose_stability_filter.sh"
  "$DIR/pose_stability_filter_beta.sh"
  "$DIR/thermal_pose_monitor.sh"
  "$DIR/relocalization_ready_notify.sh"
  "$DIR/diagnose_logger.sh"
  "ros2 launch my_static_tf"
  "ros2 launch fast_lio"
  "ros2 launch livox_ros_driver2"
  "ros2 launch rc_serial_driver"
  "ros2 launch lidar_map_localization"
  "ros2 launch lidar_localization"
  "ros2 run robot_localization"
  "ros2 run lidar_localization canfd_pose_sender"
  "canfd_tx_monitor.sh"
  "ros2 run lidar_localization pose_filter"
  "ros2 run lidar_localization health_monitor"
  "fastlio_mapping"
  "livox_ros_driver2_node"
  "rc_serial_driver_node"
  "can_pose_monitor.py"
  "canfd_pose_sender"
  "/lidar_localization/pose_filter"
  "/lidar_localization/health_monitor"
  "ekf_node"
  "static_transform_publisher"
  "lidar_xyzr.py"
  "map_localizer"
  "pose_from_odom"
  "pose_from_tf"
  "continuous_robot_pose"
  "pose_stability_filter"
  "thermal_pose_monitor.py"
  "relocalization_ready_notify.py"
  "rviz2"
  "plotjuggler"
)

log "stopping processes recorded in /tmp/rc_autostart_*.pid"
shopt -s nullglob
pidfiles=(/tmp/rc_autostart_*.pid)
for f in "${pidfiles[@]}"; do
  pid="$(cat "$f" 2>/dev/null || true)"
  kill_pid_safely "$pid" TERM
done

log "stopping matching RC LIDAR processes"
kill_matching_processes TERM "${patterns[@]}"
close_terminal_windows_by_title

if wait_for_matching_processes_to_exit "$STOP_WAIT_SEC"; then
  log "all matching RC LIDAR processes exited after TERM"
else
  log_remaining_processes "after TERM wait ${STOP_WAIT_SEC}s"
fi

log "force killing remaining matching RC LIDAR processes"
for f in "${pidfiles[@]}"; do
  pid="$(cat "$f" 2>/dev/null || true)"
  kill_pid_safely "$pid" KILL
done
kill_matching_processes KILL "${patterns[@]}"
close_terminal_windows_by_title

if wait_for_matching_processes_to_exit "$STOP_FORCE_WAIT_SEC"; then
  log "all matching RC LIDAR processes exited after KILL"
else
  log_remaining_processes "after KILL wait ${STOP_FORCE_WAIT_SEC}s"
fi

rm -f /tmp/rc_autostart_*.pid 2>/dev/null || true
log "done"
