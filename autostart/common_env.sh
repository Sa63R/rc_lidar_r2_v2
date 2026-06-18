#!/usr/bin/env bash
set -eo pipefail

COMMON_ENV_RESTORE_NOUNSET=0
case "$-" in
  *u*)
    COMMON_ENV_RESTORE_NOUNSET=1
    set +u
    ;;
esac

AUTOSTART_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$AUTOSTART_DIR/.." && pwd)"

export RC_LIDAR_AUTOSTART="$AUTOSTART_DIR"
export RC_LIDAR_WORKSPACE="$WORKSPACE_DIR"

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export RCUTILS_LOGGING_USE_STDOUT="${RCUTILS_LOGGING_USE_STDOUT:-1}"
export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-0}"

cd "$WORKSPACE_DIR"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[common_env] ERROR: /opt/ros/humble/setup.bash not found."
  exit 2
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  echo "[common_env] ERROR: $WORKSPACE_DIR/install/setup.bash not found."
  echo "[common_env] Build first: colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble"
  exit 2
fi

# shellcheck disable=SC1091
source "$WORKSPACE_DIR/install/setup.bash"

if [ "$COMMON_ENV_RESTORE_NOUNSET" = "1" ]; then
  set -u
fi
