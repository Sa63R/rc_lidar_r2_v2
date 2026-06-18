#!/usr/bin/env bash
set -euo pipefail
# shellcheck disable=SC1091

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

if ! ip -brief addr | grep -q '192\.168\.1\.5/24'; then
  echo "[livox] WARN: no local interface has 192.168.1.5/24."
  echo "[livox] WARN: MID360_config.json currently uses host IP 192.168.1.5; Livox SDK may fail to bind until the LiDAR NIC is configured."
fi

echo "[livox] route to configured lidar:"
ip route get 192.168.1.124 2>/dev/null || true

if ip -brief addr | awk '$0 ~ /192\.168\.1\.5\/24/ { count++ } END { exit(count > 1 ? 0 : 1) }'; then
  echo "[livox] WARN: 192.168.1.5/24 exists on more than one interface."
  echo "[livox] WARN: keep it only on the physical LiDAR interface, usually eth0."
fi

exec ros2 launch livox_ros_driver2 msg_MID360_launch.py
