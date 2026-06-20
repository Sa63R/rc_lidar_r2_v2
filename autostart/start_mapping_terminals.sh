#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export RUN_REASON="${RUN_REASON:-mapping}"
export RC_LIDAR_MODE="mapping"

exec "$DIR/start_all_terminals.sh"
