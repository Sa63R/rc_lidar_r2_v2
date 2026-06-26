#!/usr/bin/env bash
set -euo pipefail
# shellcheck disable=SC1091

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

if [ "${RC_AUTOSTART_SETUP_LIVOX_NET:-1}" != "0" ]; then
  SETUP_LIVOX_NET="$RC_LIDAR_AUTOSTART/setup_livox_net.sh"
  if [ ! -x "$SETUP_LIVOX_NET" ]; then
    echo "[livox] ERROR: setup script not found or not executable: $SETUP_LIVOX_NET"
    exit 1
  fi

  echo "[livox] configuring LiDAR network before starting Livox driver..."
  "$SETUP_LIVOX_NET"
else
  echo "[livox] skipped LiDAR network setup because RC_AUTOSTART_SETUP_LIVOX_NET=0."
fi

if ! ip -brief addr | grep -q '192\.168\.1\.5/24'; then
  echo "[livox] WARN: no local interface has 192.168.1.5/24."
  echo "[livox] WARN: MID360_config.json currently uses host IP 192.168.1.5; Livox SDK may fail to bind until the LiDAR NIC is configured."
fi

LIVOX_LIDAR_IP="${LIVOX_LIDAR_IP:-${RC_LIVOX_LIDAR_IP:-192.168.1.168}}"
echo "[livox] route to configured lidar:"
ip route get "$LIVOX_LIDAR_IP" 2>/dev/null || true

if ip -brief addr | awk '$0 ~ /192\.168\.1\.5\/24/ { count++ } END { exit(count > 1 ? 0 : 1) }'; then
  echo "[livox] WARN: 192.168.1.5/24 exists on more than one interface."
  echo "[livox] WARN: keep it only on the physical LiDAR interface, currently enp4s0 on this host."
fi

exec ros2 launch livox_ros_driver2 msg_MID360_launch.py
