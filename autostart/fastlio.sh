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

choose_auto_map_save_path() {
  local dir="$RC_LIDAR_WORKSPACE/maps"
  local base="$dir/mid360_map.pcd"
  mkdir -p "$dir"

  if [ ! -e "$base" ]; then
    echo "$base"
    return 0
  fi

  local stamp
  stamp="$(date +%Y%m%d_%H%M%S)"
  local candidate="$dir/mid360_map_${stamp}.pcd"
  local index=1
  while [ -e "$candidate" ]; do
    candidate="$dir/mid360_map_${stamp}_$(printf '%02d' "$index").pcd"
    index=$((index + 1))
  done
  echo "$candidate"
}

if [ -n "${RC_FASTLIO_MAP_SAVE_PATH:-}" ]; then
  MAP_SAVE_PATH="$RC_FASTLIO_MAP_SAVE_PATH"
  MAP_SAVE_MODE="explicit"
else
  MAP_SAVE_PATH="$(choose_auto_map_save_path)"
  MAP_SAVE_MODE="auto"
fi

mkdir -p "$(dirname "$MAP_SAVE_PATH")"

if [ -n "${LOG_DIR:-}" ]; then
  echo "$MAP_SAVE_PATH" > "$LOG_DIR/fastlio_map_save_path.txt"
fi

echo "[fastlio] map_file_path=$MAP_SAVE_PATH mode=$MAP_SAVE_MODE"
exec ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360.yaml \
  rviz:="$FASTLIO_RVIZ" \
  map_file_path:="$MAP_SAVE_PATH"
