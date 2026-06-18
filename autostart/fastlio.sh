#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

sleep 2

exec ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:="${FASTLIO_RVIZ:-true}"
