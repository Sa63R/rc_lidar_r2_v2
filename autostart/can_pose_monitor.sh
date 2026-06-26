#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

cd "$RC_LIDAR_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

exec /usr/bin/env python3 -u "$RC_LIDAR_AUTOSTART/can_pose_monitor.py" \
  --topic "${RC_CAN_POSE_DEBUG_TOPIC:-/rc_serial_driver/can_pose_debug}" \
  --hz "${RC_CAN_MONITOR_PRINT_HZ:-10}"
