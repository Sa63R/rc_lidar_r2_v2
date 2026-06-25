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

# ROS 2 Humble on this board uses the system Python 3.10. If a terminal has
# conda activated, python3 may point at Miniconda Python 3.13 and rclpy imports
# fail. Keep autostart scripts isolated from the user's interactive conda state.
unset CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE
unset CONDA_EXE CONDA_SHLVL _CE_CONDA _CE_M PYTHONHOME
if [ -n "${PATH:-}" ]; then
  CLEAN_PATH=""
  OLD_IFS="$IFS"
  IFS=":"
  for PATH_ENTRY in $PATH; do
    case "$PATH_ENTRY" in
      "$HOME/miniconda3"|"$HOME/miniconda3/bin"|"$HOME/miniconda3/condabin"|"$HOME/anaconda3"|"$HOME/anaconda3/bin"|"$HOME/anaconda3/condabin")
        continue
        ;;
    esac
    if [ -z "$CLEAN_PATH" ]; then
      CLEAN_PATH="$PATH_ENTRY"
    else
      CLEAN_PATH="$CLEAN_PATH:$PATH_ENTRY"
    fi
  done
  IFS="$OLD_IFS"
  export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$CLEAN_PATH"
fi

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
