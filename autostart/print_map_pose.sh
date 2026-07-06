#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

exec /usr/bin/env python3 -u "$RC_LIDAR_AUTOSTART/print_map_pose.py" "$@"
