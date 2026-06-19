#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

if [ -x "$RC_LIDAR_AUTOSTART/can_setup.sh" ]; then
  export CAN_SETUP_ENV_FILE="${CAN_SETUP_ENV_FILE:-${LOG_DIR:-/tmp}/can_setup.env}"
  set +e
  "$RC_LIDAR_AUTOSTART/can_setup.sh"
  CAN_SETUP_RC=$?
  set -e
  if [ "$CAN_SETUP_RC" -ne 0 ]; then
    echo "[relocal] WARN: CAN setup failed with exit code $CAN_SETUP_RC; relocalization will continue."
  fi
fi

LOCALIZER_GREP="localizer"   # 用来匹配 ros2 node list 里的名字关键词
MAX_WAIT=120

echo "[serial] waiting localizer node grep: ${LOCALIZER_GREP} ..."
deadline=$((SECONDS+MAX_WAIT))
while true; do
  if ros2 node list 2>/dev/null | grep -qi "${LOCALIZER_GREP}"; then
    echo "[serial] localizer node found."
    break
  fi

  if [ $SECONDS -ge $deadline ]; then
    echo "[serial] ERROR: localizer node not found after ${MAX_WAIT}s, still trying..."
    deadline=$((SECONDS+MAX_WAIT))
  fi

  sleep 1
done

# 按你说的：localizer起来后再等2秒再启动后续
sleep 2

PCD_PATH="${PCD_PATH:-$RC_LIDAR_WORKSPACE/sources/map6/map.pcd}"

exec ros2 service call /localizer/relocalize interface/srv/Relocalize \
"{pcd_path: '$PCD_PATH', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
