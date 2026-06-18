#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"



sleep 10

exec ros2 run fastlio_world_init world_from_fastlio_init --ros-args \
  -p source_parent_frame:=camera_init \
  -p source_child_frame:=body \
  -p world_frame:=world \
  -p capture_duration:=3.0 \
  -p settle_time:=0.5 \
  -p sample_period:=0.02 \
  -p orientation_mode:=none
