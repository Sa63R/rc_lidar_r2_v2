#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "$DIR/common_env.sh"

export RC_LIDAR_UI_CONFIG="${RC_LIDAR_UI_CONFIG:-${XDG_CONFIG_HOME:-$HOME/.config}/rc_lidar_r2_v2/ui_config.env}"

exec /usr/bin/env python3 -u "$DIR/lidar_control_ui.py"
