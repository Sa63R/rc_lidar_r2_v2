#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export RC_CAN_SEND_ENABLED=0
export RC_RESTART_SCRIPT="${RC_RESTART_SCRIPT:-$DIR/restart_lidar_stack.sh}"

exec "$DIR/serial_driver.sh"
