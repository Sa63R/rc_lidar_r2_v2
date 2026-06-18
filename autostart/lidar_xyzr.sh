#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

exec /usr/bin/env python3 -u "$RC_LIDAR_AUTOSTART/lidar_xyzr.py" --topic "${LIDAR_XYZR_TOPIC:-/livox/lidar}" --hz "${LIDAR_XYZR_HZ:-10}" --max-points "${LIDAR_XYZR_MAX_POINTS:-5}"
