#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

MAP_PATH="${RC_LIDAR_MAP_PATH:-$RC_LIDAR_WORKSPACE/maps/mid360_map.pcd}"

if [ ! -s "$MAP_PATH" ]; then
  echo "[map_localizer] ERROR: map file is missing or empty: $MAP_PATH"
  echo "[map_localizer] Build and save a map first:"
  echo "[map_localizer]   RC_LIDAR_MODE=mapping ./autostart/start_all_terminals.sh"
  echo "[map_localizer]   ./autostart/save_fast_lio_map.sh"
  exit 3
fi

echo "[map_localizer] map_path=$MAP_PATH"
exec ros2 launch lidar_map_localization map_localizer.launch.py map_path:="$MAP_PATH"
