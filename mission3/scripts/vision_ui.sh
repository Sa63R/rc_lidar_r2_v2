#!/usr/bin/env bash
set -euo pipefail
TASK3_UI_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TASK3_UI_ROS_SETUP="${TASK1_ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
set +u
source "$TASK3_UI_ROS_SETUP"
if [[ -n "${TASK1_BASE_SETUP:-}" ]]; then source "$TASK1_BASE_SETUP"; fi
source "$TASK3_UI_ROOT/install/setup.bash"
set -u
exec ros2 run task1_fusion vision_ui "$@"
