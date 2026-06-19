#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

sleep 2

if [ -z "${FASTLIO_RVIZ:-}" ]; then
  if [ "${RC_AUTOSTART_HEADLESS:-0}" = "1" ]; then
    FASTLIO_RVIZ=false
  else
    FASTLIO_RVIZ=true
  fi
fi

echo "[fastlio] FASTLIO_RVIZ=$FASTLIO_RVIZ"
exec ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:="$FASTLIO_RVIZ"
