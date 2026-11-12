#!/usr/bin/env bash
# Keep the car still during initialization. Existing LiDAR/FAST-LIO stay running.
set -euo pipefail
TASK3_REL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TASK3_REL_BASE="$TASK3_REL_ROOT/config/car_task3_local.yaml"
if [[ "${1:-}" == --base-config ]]; then
  TASK3_REL_BASE="$2"
  shift 2
fi
set +u
source "${TASK1_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
source "${TASK1_BASE_SETUP:-/opt/ros/jazzy/setup.bash}"
source "$TASK3_REL_ROOT/install/setup.bash"
set -u
TASK3_REL_NODES="$(ros2 node list)"
while IFS= read -r TASK3_REL_NODE; do
  case "$TASK3_REL_NODE" in
    /task3_corner|/task3_vision)
      printf '任务三已运行。先在原任务三终端按Ctrl+C，再运行本入口；驱动和FAST-LIO保持运行。\n' >&2
      exit 2 ;;
  esac
done <<< "$TASK3_REL_NODES"
TASK3_REL_LINK="${TASK3_ACTIVE_CONFIG_LINK:-$TASK3_REL_ROOT/config/car_task3_start.yaml}"
if [[ -e "$TASK3_REL_LINK" && ! -L "$TASK3_REL_LINK" ]]; then
  printf 'config/car_task3_start.yaml已是普通文件，拒绝覆盖。\n' >&2
  exit 2
fi
TASK3_REL_OUT="${TASK3_START_RESULTS_DIR:-$TASK3_REL_ROOT/results}/task3_start_$(date +%Y%m%d_%H%M%S)_$$"
printf '请保持车不动：将当前角点设为XY原点，当前图示+X设为固定+X。\n'
TASK3_REL_EXTRA=()
for TASK3_REL_ARG in "$@"; do TASK3_REL_EXTRA+=(--launch-arg "$TASK3_REL_ARG"); done
exec /usr/bin/python3 "$TASK3_REL_ROOT/tools/task3_set_start.py" \
  --base-config "$TASK3_REL_BASE" --out-dir "$TASK3_REL_OUT" --launch \
  --active-config-link "$TASK3_REL_LINK" "${TASK3_REL_EXTRA[@]}"
