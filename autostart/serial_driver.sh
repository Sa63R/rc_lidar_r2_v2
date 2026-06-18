#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

CAN_INTERFACE="${CAN_INTERFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"

setup_can_if_possible() {
  if ! ip link show "$CAN_INTERFACE" >/dev/null 2>&1; then
    echo "[serial] WARN: CAN interface $CAN_INTERFACE not found."
    return 0
  fi

  if ip -details link show "$CAN_INTERFACE" | grep -q "state UP"; then
    echo "[serial] $CAN_INTERFACE is already UP."
    return 0
  fi

  if sudo -n true >/dev/null 2>&1; then
    echo "[serial] Bringing up $CAN_INTERFACE at ${CAN_BITRATE} bit/s..."
    sudo ip link set "$CAN_INTERFACE" down || true
    sudo ip link set "$CAN_INTERFACE" type can bitrate "$CAN_BITRATE" restart-ms 100
    sudo ip link set "$CAN_INTERFACE" up
  else
    echo "[serial] WARN: $CAN_INTERFACE is DOWN and sudo is not available without a password."
    echo "[serial] WARN: run manually before starting, or configure passwordless sudo:"
    echo "[serial]       sudo ip link set $CAN_INTERFACE down"
    echo "[serial]       sudo ip link set $CAN_INTERFACE type can bitrate $CAN_BITRATE restart-ms 100"
    echo "[serial]       sudo ip link set $CAN_INTERFACE up"
  fi
}

setup_can_if_possible

sleep 3

exec ros2 launch rc_serial_driver serial_driver.launch.py
