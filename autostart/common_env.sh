#!/usr/bin/env bash
set -eo pipefail

COMMON_ENV_RESTORE_NOUNSET=0
case "$-" in
  *u*)
    COMMON_ENV_RESTORE_NOUNSET=1
    set +u
    ;;
esac

SCRIPT_AUTOSTART_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_ROOT="$(cd "$SCRIPT_AUTOSTART_DIR/.." && pwd)"

export RC_LIDAR_ROOT="${RC_LIDAR_ROOT:-$DEFAULT_ROOT}"
export RC_LIDAR_AUTOSTART="${RC_LIDAR_AUTOSTART:-$RC_LIDAR_ROOT/autostart}"
export RC_LIDAR_WORKSPACE="${RC_LIDAR_WORKSPACE:-$RC_LIDAR_ROOT}"

AUTOSTART_DIR="$RC_LIDAR_AUTOSTART"
WORKSPACE_DIR="$RC_LIDAR_WORKSPACE"

# ROS 2 uses the distribution system Python (Jazzy: Python 3.12). If a terminal has
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

# Hikrobot MVS exports an older libusb globally. PCL 1.14 needs the system
# libusb_set_option symbol. Isolate this ROS process tree; do not edit MVS.
RC_CLEAN_LD_PATH=""
IFS=: read -r -a RC_LD_ENTRIES <<< "${LD_LIBRARY_PATH:-}"
for RC_LD_ENTRY in "${RC_LD_ENTRIES[@]}"; do
  case "$RC_LD_ENTRY" in /opt/MVS/lib/*|"") continue ;; esac
  RC_CLEAN_LD_PATH="${RC_CLEAN_LD_PATH:+$RC_CLEAN_LD_PATH:}$RC_LD_ENTRY"
done
export LD_LIBRARY_PATH="/usr/local/lib${RC_CLEAN_LD_PATH:+:$RC_CLEAN_LD_PATH}"
export RCUTILS_LOGGING_USE_STDOUT="${RCUTILS_LOGGING_USE_STDOUT:-1}"
export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-0}"

rc_bool() {
  case "${1:-false}" in
    1|true|TRUE|yes|YES|on|ON) echo "true" ;;
    *) echo "false" ;;
  esac
}

export RC_ROS_DISTRO="${RC_ROS_DISTRO:-jazzy}"
export RC_ROS_SETUP="${RC_ROS_SETUP:-/opt/ros/$RC_ROS_DISTRO/setup.bash}"

cd "$WORKSPACE_DIR"

if [ ! -f "$RC_ROS_SETUP" ]; then
  echo "[common_env] ERROR: $RC_ROS_SETUP not found."
  exit 2
fi

# shellcheck disable=SC1091
source "$RC_ROS_SETUP"

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  echo "[common_env] ERROR: $WORKSPACE_DIR/install/setup.bash not found."
  echo "[common_env] Build first: colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DCMAKE_CXX_STANDARD=17"
  exit 2
fi

# shellcheck disable=SC1091
source "$WORKSPACE_DIR/install/setup.bash"

if [ "$COMMON_ENV_RESTORE_NOUNSET" = "1" ]; then
  set -u
fi
