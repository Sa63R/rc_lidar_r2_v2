#!/usr/bin/env bash
# Isolated synthetic demo: never connects to the real car or starts its driver.
set -euo pipefail
TASK3_DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${TASK3_DEMO_DOMAIN_ID:-93}"
set +u
source /opt/ros/jazzy/setup.bash
source "$TASK3_DEMO_ROOT/install/setup.bash"
set -u
exec /usr/bin/python3 "$TASK3_DEMO_ROOT/tools/task3_stationary_demo.py" "$@"
