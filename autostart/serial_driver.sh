#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

CAN_INTERFACE="${CAN_INTERFACE:-}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
CAN_SETUP_SCRIPT="$RC_LIDAR_AUTOSTART/can_setup.sh"
CAN_SETUP_ENV_FILE="${CAN_SETUP_ENV_FILE:-${LOG_DIR:-/tmp}/can_setup.env}"
export CAN_SETUP_ENV_FILE

setup_can_if_possible() {
  if [ ! -x "$CAN_SETUP_SCRIPT" ]; then
    echo "[serial] WARN: CAN setup script not found: $CAN_SETUP_SCRIPT"
    return 0
  fi

  set +e
  "$CAN_SETUP_SCRIPT"
  local setup_rc=$?
  set -e

  if [ -f "$CAN_SETUP_ENV_FILE" ]; then
    # shellcheck disable=SC1090
    source "$CAN_SETUP_ENV_FILE"
    export CAN_INTERFACE
    export CAN_BITRATE
    export CAN_RESTART_MS="${CAN_RESTART_MS:-100}"
  fi

  if [ "$setup_rc" -ne 0 ]; then
    if [ "${RC_CAN_ALLOW_MISSING:-0}" = "1" ]; then
      echo "[serial] WARN: CAN setup failed with exit code $setup_rc, continuing because RC_CAN_ALLOW_MISSING=1."
      return 0
    fi

    echo "[serial] ERROR: CAN setup failed with exit code $setup_rc."
    echo "[serial] ERROR: rc_serial_driver will not be launched because no usable CAN interface was found."
    echo "[serial] HINT: replug USB-CAN, then run: $CAN_SETUP_SCRIPT"
    exit "$setup_rc"
  fi
}

setup_can_if_possible

sleep 3

if [ -z "${RC_CAN_POSE_SOURCE:-}" ]; then
  if [ "${RC_LIDAR_MODE:-tracking}" = "relocalization" ]; then
    RC_CAN_POSE_SOURCE="pose_topic"
  else
    RC_CAN_POSE_SOURCE="tf"
  fi
fi
if [ -z "${RC_CAN_POSE_TOPIC:-}" ]; then
  if [ "${RC_LIDAR_MODE:-tracking}" = "relocalization" ] && [ "${RC_POSE_FILTER:-1}" != "0" ]; then
    RC_CAN_POSE_TOPIC="${RC_POSE_FILTER_OUTPUT_TOPIC:-/lidar/pose_filtered}"
  else
    RC_CAN_POSE_TOPIC="/lidar/pose"
  fi
fi
export RC_CAN_POSE_SOURCE
export RC_CAN_POSE_TOPIC

echo "[serial] Launching rc_serial_driver with CAN_INTERFACE=${CAN_INTERFACE:-unset}, CAN_BITRATE=$CAN_BITRATE"
echo "[serial] pose_source=$RC_CAN_POSE_SOURCE pose_topic=$RC_CAN_POSE_TOPIC"
exec ros2 launch rc_serial_driver serial_driver.launch.py
