#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

POSE_SOURCE="${RC_ROBOT_POSE_SOURCE:-odom}"

case "$POSE_SOURCE" in
  odom)
    ZERO_FRAME="${RC_POSE_ZERO_FRAME_ID:-robot_origin}"
    echo "[pose_interface] relocalization mode: input=/map_lidar/odom publishes /robot/pose in $ZERO_FRAME"
    exec ros2 launch lidar_localization lidar_pose_from_odom.launch.py \
      input_odom_topic:=/map_lidar/odom \
      zero_on_first_pose:=false \
      zero_frame_id:="$ZERO_FRAME" \
      reference_pose_enabled:=true \
      reference_frame_id:="$ZERO_FRAME"
    ;;
  tf)
    TARGET_FRAME="${RC_ROBOT_POSE_TARGET_FRAME:-robot_origin}"
    SOURCE_FRAME="${RC_ROBOT_POSE_SOURCE_FRAME:-robot_base_highrate}"
    Z_SOURCE_FRAME="${RC_ROBOT_POSE_Z_SOURCE_FRAME:-robot_base}"
    echo "[pose_interface] relocalization mode: TF $TARGET_FRAME -> $SOURCE_FRAME publishes /robot/pose"
    exec ros2 run lidar_localization pose_from_tf --ros-args \
      -p target_frame:="$TARGET_FRAME" \
      -p source_frame:="$SOURCE_FRAME" \
      -p z_source_frame:="$Z_SOURCE_FRAME" \
      -p publish_rate:="${RC_ROBOT_POSE_HZ:-50.0}" \
      -p max_tf_age_sec:="${RC_ROBOT_POSE_MAX_TF_AGE:-0.0}"
    ;;
  *)
    echo "[pose_interface] ERROR: unknown RC_ROBOT_POSE_SOURCE=$POSE_SOURCE. Use odom or tf." >&2
    exit 2
    ;;
esac
