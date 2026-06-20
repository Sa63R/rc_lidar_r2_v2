#!/usr/bin/env bash
set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$DIR/.." && pwd)"
QUIET=0

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

should_kill_cmd() {
  local cmd="$1"

  case "$cmd" in
    *"$WORKSPACE_DIR"*|*"$DIR"*)
      return 0
      ;;
    *"ros2 launch my_static_tf "*|*"ros2 launch fast_lio "*|*"ros2 launch livox_ros_driver2 "*|*"ros2 launch rc_serial_driver "*|*"ros2 launch lidar_map_localization "*|*"ros2 launch lidar_localization "*)
      return 0
      ;;
    *"ros2 run robot_localization "*|*"ros2 run fastlio_world_init "*)
      return 0
      ;;
    *"fastlio_mapping"*|*"livox_ros_driver2_node"*|*"rc_serial_driver_node"*|*"ekf_node"*)
      return 0
      ;;
    *"world_from_fastlio_init"*|*"robot_tf_offsets"*|*"lidar_tf_offset"*|*"lidar_xyzr.py"*|*"map_localizer"*|*"pose_from_odom"*)
      return 0
      ;;
    *"thermal_pose_monitor.sh"*|*"thermal_pose_monitor.py"*)
      return 0
      ;;
    *"rotation_extrinsic_calibrator"*)
      return 0
      ;;
    *"static_transform_publisher"*base_link*|*"static_transform_publisher"*body*|*"static_transform_publisher"*init*)
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
    "ROTATION_CALIB"
    "MAP_LOCALIZER"
    "POSE_INTERFACE"
    "POSE_FILTER"
    "THERMAL_MONITOR"
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
  "$DIR/thermal_pose_monitor.sh"
  "$DIR/diagnose_logger.sh"
  "ros2 launch my_static_tf"
  "ros2 launch fast_lio"
  "ros2 launch livox_ros_driver2"
  "ros2 launch rc_serial_driver"
  "ros2 launch lidar_map_localization"
  "ros2 launch lidar_localization"
  "ros2 run robot_localization"
  "ros2 run fastlio_world_init"
  "fastlio_mapping"
  "livox_ros_driver2_node"
  "rc_serial_driver_node"
  "ekf_node"
  "static_transform_publisher"
  "world_from_fastlio_init"
  "robot_tf_offsets"
  "lidar_tf_offset"
  "rotation_extrinsic_calibrator"
  "lidar_xyzr.py"
  "map_localizer"
  "pose_from_odom"
  "pose_stability_filter"
  "thermal_pose_monitor.py"
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

sleep 2

log "force killing remaining matching RC LIDAR processes"
for f in "${pidfiles[@]}"; do
  pid="$(cat "$f" 2>/dev/null || true)"
  kill_pid_safely "$pid" KILL
done
kill_matching_processes KILL "${patterns[@]}"
close_terminal_windows_by_title

rm -f /tmp/rc_autostart_*.pid 2>/dev/null || true
log "done"
