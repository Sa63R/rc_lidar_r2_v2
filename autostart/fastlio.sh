#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

sleep 2

if [ -z "${FASTLIO_RVIZ:-}" ]; then
  FASTLIO_RVIZ=false
fi

rviz_requested() {
  case "${FASTLIO_RVIZ:-false}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

rviz_force_without_display() {
  case "${FASTLIO_RVIZ_ALLOW_HEADLESS:-0}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

if rviz_requested && ! rviz_force_without_display; then
  if [ -z "${DISPLAY:-}" ]; then
    echo "[fastlio] WARN: FASTLIO_RVIZ=$FASTLIO_RVIZ but DISPLAY is empty; disabling RViz."
    FASTLIO_RVIZ=false
  elif command -v xdpyinfo >/dev/null 2>&1 && ! xdpyinfo >/dev/null 2>&1; then
    echo "[fastlio] WARN: FASTLIO_RVIZ=$FASTLIO_RVIZ but DISPLAY=${DISPLAY:-} is not reachable; disabling RViz."
    FASTLIO_RVIZ=false
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

FASTLIO_LAUNCH_ARGS=(
  "config_file:=mid360.yaml"
  "rviz:=$FASTLIO_RVIZ"
  "map_file_path:=$MAP_SAVE_PATH"
)

if [ -n "${RC_FASTLIO_FILTER_SIZE_SURF:-}" ]; then
  FASTLIO_LAUNCH_ARGS+=("filter_size_surf:=$RC_FASTLIO_FILTER_SIZE_SURF")
fi
if [ -n "${RC_FASTLIO_FILTER_SIZE_MAP:-}" ]; then
  FASTLIO_LAUNCH_ARGS+=("filter_size_map:=$RC_FASTLIO_FILTER_SIZE_MAP")
fi
if [ -n "${RC_FASTLIO_DET_RANGE:-}" ]; then
  FASTLIO_LAUNCH_ARGS+=("det_range:=$RC_FASTLIO_DET_RANGE")
fi

echo "[fastlio] filter_size_surf=${RC_FASTLIO_FILTER_SIZE_SURF:-yaml_default} filter_size_map=${RC_FASTLIO_FILTER_SIZE_MAP:-yaml_default} det_range=${RC_FASTLIO_DET_RANGE:-yaml_default}"
exec ros2 launch fast_lio mapping.launch.py "${FASTLIO_LAUNCH_ARGS[@]}"
