#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

POSE_SOURCE="${RC_ROBOT_POSE_SOURCE:-odom}"

case "$POSE_SOURCE" in
  odom)
    ZERO_FRAME="${RC_POSE_ZERO_FRAME_ID:-robot_origin}"
    YAW_FUSION="${RC_ROBOT_POSE_YAW_FUSION:-1}"
    FAST_ODOM_TOPIC="${RC_ROBOT_POSE_FAST_ODOM_TOPIC:-/Odometry}"
    FAST_YAW_PUBLISH="${RC_ROBOT_POSE_YAW_FUSION_PUBLISH_ON_FAST_ODOM:-1}"
    FAST_YAW_MAX_AGE="${RC_ROBOT_POSE_YAW_FUSION_MAX_FAST_AGE:-0.50}"
    MAP_CORRECTION_ALPHA="${RC_ROBOT_POSE_YAW_FUSION_MAP_CORRECTION_ALPHA:-0.05}"
    echo "[pose_interface] relocalization mode: input=/map_lidar/odom publishes /robot/pose in $ZERO_FRAME"
    echo "[pose_interface] yaw_fusion=$YAW_FUSION fast_odom_topic=$FAST_ODOM_TOPIC fast_publish=$FAST_YAW_PUBLISH max_fast_age=$FAST_YAW_MAX_AGE map_correction_alpha=$MAP_CORRECTION_ALPHA"
    exec ros2 launch lidar_localization lidar_pose_from_odom.launch.py \
      input_odom_topic:=/map_lidar/odom \
      zero_on_first_pose:=false \
      zero_frame_id:="$ZERO_FRAME" \
      reference_pose_enabled:=true \
      reference_frame_id:="$ZERO_FRAME" \
      output_stamp_mode:="${RC_ROBOT_POSE_STAMP_MODE:-now}" \
      yaw_fusion_enabled:="$YAW_FUSION" \
      fast_odom_topic:="$FAST_ODOM_TOPIC" \
      yaw_fusion_publish_on_fast_odom:="$FAST_YAW_PUBLISH" \
      yaw_fusion_max_fast_age_sec:="$FAST_YAW_MAX_AGE" \
      yaw_fusion_map_correction_alpha:="$MAP_CORRECTION_ALPHA"
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
