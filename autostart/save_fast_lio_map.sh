#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

MAP_SAVE_PATH="${RC_FASTLIO_MAP_SAVE_PATH:-$RC_LIDAR_WORKSPACE/maps/mid360_map.pcd}"
mkdir -p "$(dirname "$MAP_SAVE_PATH")"

echo "[map_save] target path from FAST-LIO map_file_path: $MAP_SAVE_PATH"
echo "[map_save] waiting for /map_save service..."

for _ in $(seq 1 60); do
  if ros2 service type /map_save 2>/dev/null | grep -q "std_srvs/srv/Trigger"; then
    break
  fi
  sleep 1
done

if ! ros2 service type /map_save 2>/dev/null | grep -q "std_srvs/srv/Trigger"; then
  echo "[map_save] ERROR: /map_save service not found. Start FAST-LIO first."
  exit 3
fi

echo "[map_save] calling /map_save..."
ros2 service call /map_save std_srvs/srv/Trigger "{}"
echo "[map_save] done. Check: $MAP_SAVE_PATH"
