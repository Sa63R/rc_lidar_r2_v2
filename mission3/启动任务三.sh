#!/usr/bin/env bash
set -euo pipefail
TASK3_ENTRY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$TASK3_ENTRY_ROOT/bin/task3" run "$@"
