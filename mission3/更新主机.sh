#!/usr/bin/env bash
set -euo pipefail
TASK3_UPDATE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$TASK3_UPDATE_ROOT/tools/install_update.py" --archive "$TASK3_UPDATE_ROOT/../task3_corner_vision_20261121_r4.tar.gz" --target "${1:-$HOME/rc_mission3/mission3}"
