#!/usr/bin/env bash
# Fixed-map localization with automatic global initialization from live clouds.
set -euo pipefail
TASK3_MAP_FILE="${1:?用法：bash scripts/start_car_task3_map_localizer.sh 地图.pcd}"
shift
if [[ ! -s "$TASK3_MAP_FILE" ]]; then
  printf '地图不存在或为空：%s\n' "$TASK3_MAP_FILE" >&2
  exit 2
fi
TASK3_MAP_FILE="$(realpath -- "$TASK3_MAP_FILE")"
TASK3_MAP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source "${TASK1_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [[ -n "${TASK1_BASE_SETUP:-}" ]]; then
  source "$TASK1_BASE_SETUP"
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
set -u
if ! ros2 pkg prefix lidar_map_localization >/dev/null 2>&1; then
  printf '当前环境没有lidar_map_localization。原r3安装包只有驱动和FAST-LIO；需先部署旧地图定位包。\n' >&2
  exit 2
fi
printf '固定地图：%s；启用自动整图搜索，无需鼠标指定位置。\n' "$TASK3_MAP_FILE"
exec /usr/bin/python3 "$TASK3_MAP_ROOT/tools/task3_map_runtime.py" --map "$TASK3_MAP_FILE" "$@"
