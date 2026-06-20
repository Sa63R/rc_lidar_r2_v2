#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

sleep 2

if [ -z "${FASTLIO_RVIZ:-}" ]; then
  if [ "${RC_AUTOSTART_HEADLESS:-0}" = "1" ]; then
    FASTLIO_RVIZ=false
  else
    FASTLIO_RVIZ=true
  fi
fi

echo "[fastlio] FASTLIO_RVIZ=$FASTLIO_RVIZ"
MAP_SAVE_PATH="${RC_FASTLIO_MAP_SAVE_PATH:-$RC_LIDAR_WORKSPACE/maps/mid360_map.pcd}"
mkdir -p "$(dirname "$MAP_SAVE_PATH")"

echo "[fastlio] map_file_path=$MAP_SAVE_PATH"
exec ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360.yaml \
  rviz:="$FASTLIO_RVIZ" \
  map_file_path:="$MAP_SAVE_PATH"
