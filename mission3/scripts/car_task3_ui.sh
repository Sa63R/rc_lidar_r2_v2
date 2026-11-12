#!/usr/bin/env bash
set -euo pipefail
TASK3_PANEL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export TASK1_ROS_SETUP="${TASK1_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
export TASK1_BASE_SETUP="${TASK1_BASE_SETUP:-/opt/ros/jazzy/setup.bash}"
exec bash "$TASK3_PANEL_ROOT/scripts/vision_ui.sh" \
  --corner-config "$TASK3_PANEL_ROOT/config/car_task3_local.yaml" \
  --relative-start-script "$TASK3_PANEL_ROOT/scripts/start_car_task3_relative.sh" "$@"
