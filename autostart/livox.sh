#!/usr/bin/env bash
set -euo pipefail
# shellcheck disable=SC1091

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

if ! ip -brief addr | grep -q '192\.168\.1\.5/24'; then
  echo "[livox] WARN: no local interface has 192.168.1.5/24."
  echo "[livox] WARN: MID360_config.json currently uses host IP 192.168.1.5; Livox SDK may fail to bind until the LiDAR NIC is configured."
fi

exec ros2 launch livox_ros_driver2 msg_MID360_launch.py
