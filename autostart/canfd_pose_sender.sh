#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

CAN_DEVICE_DEFAULT="/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_206D35C24831-if00"
CAN_DEVICE="${CAN_DEVICE:-$CAN_DEVICE_DEFAULT}"
CAN_SEND_RATE="${CAN_SEND_RATE:-100.0}"
CAN_POSE_TOPIC="${CAN_POSE_TOPIC:-${RC_CAN_POSE_TOPIC:-/robot/pose_filtered}}"
CAN_POSE_TIMEOUT_SEC="${CAN_POSE_TIMEOUT_SEC:-${RC_CAN_POSE_TIMEOUT_SEC:-0.5}}"
CAN_DEBUG_CSV="${CAN_DEBUG_CSV:-${RC_CAN_DEBUG_CSV:-0}}"
CAN_DEBUG_CSV_PATH="${CAN_DEBUG_CSV_PATH:-${RC_CAN_DEBUG_CSV_PATH:-}}"
CAN_DEBUG_LOG_PERIOD_SEC="${CAN_DEBUG_LOG_PERIOD_SEC:-${RC_CAN_DEBUG_LOG_PERIOD_SEC:-1.0}}"
CANFD_RESTART_ENABLED="${CANFD_RESTART_ENABLED:-${RC_CANFD_RESTART_ENABLED:-1}}"
CANFD_RESTART_CAN_ID="${CANFD_RESTART_CAN_ID:-${RC_CANFD_RESTART_CAN_ID:-0x30}}"
CANFD_RESTART_PAYLOAD_HEX="${CANFD_RESTART_PAYLOAD_HEX:-${RC_CANFD_RESTART_PAYLOAD_HEX:-A55A01FE00000000}}"
CANFD_RESTART_SCRIPT="${CANFD_RESTART_SCRIPT:-${RC_RESTART_SCRIPT:-$RC_LIDAR_AUTOSTART/restart_lidar_stack.sh}}"
CANFD_RESTART_DEBOUNCE_SEC="${CANFD_RESTART_DEBOUNCE_SEC:-${RC_CAN_RESTART_DEBOUNCE_SEC:-8.0}}"
CANFD_RESTART_CAN_ID_PARAM="$(printf '%d' "$CANFD_RESTART_CAN_ID")"

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

if truthy "$CANFD_RESTART_ENABLED"; then
  CANFD_RESTART_ENABLED_PARAM=true
else
  CANFD_RESTART_ENABLED_PARAM=false
fi

echo "CAN FD pose topic: $CAN_POSE_TOPIC"
echo "CAN FD pose timeout: ${CAN_POSE_TIMEOUT_SEC}s"
echo "CAN FD debug CSV: ${CAN_DEBUG_CSV_PATH:-off}"
echo "CAN FD restart listener: $CANFD_RESTART_ENABLED id=$CANFD_RESTART_CAN_ID payload=$CANFD_RESTART_PAYLOAD_HEX"

exec ros2 run lidar_localization canfd_pose_sender --ros-args \
  -p pose_topic:="$CAN_POSE_TOPIC" \
  -p pose_timeout_sec:="$CAN_POSE_TIMEOUT_SEC" \
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
  -p allow_serial_fallback:=false \
  -p restart_enabled:="$CANFD_RESTART_ENABLED_PARAM" \
  -p restart_can_id:="$CANFD_RESTART_CAN_ID_PARAM" \
  -p restart_payload_hex:="$CANFD_RESTART_PAYLOAD_HEX" \
  -p restart_script:="$CANFD_RESTART_SCRIPT" \
  -p restart_debounce_sec:="$CANFD_RESTART_DEBOUNCE_SEC"
