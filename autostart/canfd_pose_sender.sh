#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

CAN_DEVICE_DEFAULT="/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_206D35C24831-if00"
CAN_DEVICE="${CAN_DEVICE:-$CAN_DEVICE_DEFAULT}"
CAN_SEND_RATE="${CAN_SEND_RATE:-100.0}"
CAN_POSE_TOPIC="${CAN_POSE_TOPIC:-${RC_CAN_POSE_TOPIC:-/robot/pose_filtered}}"
CAN_DEBUG_CSV="${CAN_DEBUG_CSV:-${RC_CAN_DEBUG_CSV:-0}}"
CAN_DEBUG_CSV_PATH="${CAN_DEBUG_CSV_PATH:-${RC_CAN_DEBUG_CSV_PATH:-}}"
CAN_DEBUG_LOG_PERIOD_SEC="${CAN_DEBUG_LOG_PERIOD_SEC:-${RC_CAN_DEBUG_LOG_PERIOD_SEC:-1.0}}"

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
  esac
  return 1
}

if truthy "$CAN_DEBUG_CSV" && [ -z "$CAN_DEBUG_CSV_PATH" ] && [ -n "${LOG_DIR:-}" ]; then
  CAN_DEBUG_CSV_PATH="$LOG_DIR/canfd_pose_payload.csv"
fi

echo "CAN FD pose topic: $CAN_POSE_TOPIC"
echo "CAN FD debug CSV: ${CAN_DEBUG_CSV_PATH:-off}"

exec ros2 run lidar_localization canfd_pose_sender --ros-args \
  -p pose_topic:="$CAN_POSE_TOPIC" \
  -p debug_csv_path:="$CAN_DEBUG_CSV_PATH" \
  -p debug_log_period_sec:="$CAN_DEBUG_LOG_PERIOD_SEC" \
  -p transport:=slcan_fd \
  -p serial_device:="$CAN_DEVICE" \
  -p can_id:=18 \
  -p send_rate_hz:="$CAN_SEND_RATE" \
  -p enable_brs:=true \
  -p auto_retransmit:=true \
  -p reconnect_on_error:=true \
  -p reconnect_period_sec:=0.5 \
  -p allow_serial_fallback:=true
