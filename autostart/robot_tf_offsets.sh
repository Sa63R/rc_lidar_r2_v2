#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

exec ros2 run fastlio_world_init robot_tf_offsets \
  --publish-rate "${TF_COMPARE_ROBOT_HZ:-30}" \
  --warn-period "${TF_COMPARE_WARN_PERIOD:-2}"
