#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

read_running_map_path() {
  local value
  value="$(ros2 param get /laser_mapping map_file_path 2>/dev/null || true)"
  value="${value#String value is: }"
  value="${value#string value is: }"
  if [ -n "$value" ] && [ "$value" != "Not set" ]; then
    echo "$value"
  fi
}

MAP_SAVE_PATH="${RC_FASTLIO_MAP_SAVE_PATH:-}"
if [ -z "$MAP_SAVE_PATH" ]; then
  MAP_SAVE_PATH="$(read_running_map_path)"
fi
if [ -z "$MAP_SAVE_PATH" ] && [ -f "$RC_LIDAR_WORKSPACE/logs/autostart/latest/fastlio_map_save_path.txt" ]; then
  MAP_SAVE_PATH="$(cat "$RC_LIDAR_WORKSPACE/logs/autostart/latest/fastlio_map_save_path.txt")"
fi
if [ -z "$MAP_SAVE_PATH" ]; then
  MAP_SAVE_PATH="$RC_LIDAR_WORKSPACE/maps/mid360_map.pcd"
fi

mkdir -p "$(dirname "$MAP_SAVE_PATH")"

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

RUNNING_MAP_SAVE_PATH="$(read_running_map_path)"
if [ -n "$RUNNING_MAP_SAVE_PATH" ]; then
  MAP_SAVE_PATH="$RUNNING_MAP_SAVE_PATH"
fi

echo "[map_save] target path from FAST-LIO map_file_path: $MAP_SAVE_PATH"
echo "[map_save] calling /map_save..."
RESPONSE="$(ros2 service call /map_save std_srvs/srv/Trigger "{}")"
printf '%s\n' "$RESPONSE"

SAVED_PATH="$(printf '%s\n' "$RESPONSE" | sed -n "s/.*message='Map saved to \\(.*\\)'.*/\\1/p" | tail -n 1)"
if [ -n "$SAVED_PATH" ]; then
  echo "[map_save] done. Saved: $SAVED_PATH"
else
  echo "[map_save] done. Requested target was: $MAP_SAVE_PATH"
fi
