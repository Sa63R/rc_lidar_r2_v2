#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

echo "[EKF] Waiting 3 seconds before start..."
sleep 3

if ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
  echo "[EKF] WARN: robot_localization is not installed in this ROS environment."
  echo "[EKF] WARN: falling back to a static identity TF body -> body_highrate."
  echo "[EKF] WARN: install ros-humble-robot-localization to enable real high-rate EKF output."
  exec ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id body \
    --child-frame-id body_highrate
fi

echo "[EKF] Starting robot_localization ekf_node..."

exec ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file "$RC_LIDAR_WORKSPACE/src/my_static_tf/config/ekf_fastlio.yaml"
