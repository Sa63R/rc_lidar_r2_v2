#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

CAN_DEVICE_DEFAULT="/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_206D35C24831-if00"
CAN_DEVICE="${CAN_DEVICE:-${RC_FDCAN_DEVICE:-$CAN_DEVICE_DEFAULT}}"
FDCAN_GATEWAY_SOCKET="${FDCAN_GATEWAY_SOCKET:-${RC_FDCAN_GATEWAY_SOCKET:-/tmp/rc_fdcan_gateway.sock}}"
FDCAN_GATEWAY_LOCK="${FDCAN_GATEWAY_LOCK:-${RC_FDCAN_GATEWAY_LOCK:-/tmp/rc_fdcan_gateway.lock}}"
FDCAN_NOMINAL_SPEED_COMMAND="${FDCAN_NOMINAL_SPEED_COMMAND:-${RC_FDCAN_NOMINAL_SPEED_COMMAND:-S8}}"
FDCAN_DATA_SPEED_COMMAND="${FDCAN_DATA_SPEED_COMMAND:-${RC_FDCAN_DATA_SPEED_COMMAND:-Y2}}"
FDCAN_RECONNECT_PERIOD_SEC="${FDCAN_RECONNECT_PERIOD_SEC:-${RC_FDCAN_RECONNECT_PERIOD_SEC:-0.5}}"
FDCAN_RECONNECT_LOG_PERIOD_SEC="${FDCAN_RECONNECT_LOG_PERIOD_SEC:-${RC_FDCAN_RECONNECT_LOG_PERIOD_SEC:-2.0}}"

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
  esac
  return 1
}

if truthy "${FDCAN_AUTO_RETRANSMIT:-${RC_FDCAN_AUTO_RETRANSMIT:-1}}"; then
  AUTO_RETRANSMIT_ARG="--auto-retransmit"
else
  AUTO_RETRANSMIT_ARG="--no-auto-retransmit"
fi

echo "FDCAN gateway socket: $FDCAN_GATEWAY_SOCKET"
echo "FDCAN gateway device: $CAN_DEVICE"
echo "FDCAN nominal speed command: $FDCAN_NOMINAL_SPEED_COMMAND"
echo "FDCAN data speed command: $FDCAN_DATA_SPEED_COMMAND"
echo "FDCAN reconnect period: ${FDCAN_RECONNECT_PERIOD_SEC}s"

exec ros2 run lidar_localization fdcan_gateway \
  --socket-path "$FDCAN_GATEWAY_SOCKET" \
  --lock-path "$FDCAN_GATEWAY_LOCK" \
  --serial-device "$CAN_DEVICE" \
  --nominal-speed-command "$FDCAN_NOMINAL_SPEED_COMMAND" \
  --data-speed-command "$FDCAN_DATA_SPEED_COMMAND" \
  "$AUTO_RETRANSMIT_ARG" \
  --reconnect-period-sec "$FDCAN_RECONNECT_PERIOD_SEC" \
  --reconnect-log-period-sec "$FDCAN_RECONNECT_LOG_PERIOD_SEC"
