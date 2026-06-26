#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

FAST_ODOM_TOPIC="${RC_CONTINUOUS_POSE_FAST_ODOM_TOPIC:-/Odometry}"
MAP_ODOM_TOPIC="${RC_CONTINUOUS_POSE_MAP_ODOM_TOPIC:-/map_lidar/odom}"
QUALITY_TOPIC="${RC_CONTINUOUS_POSE_QUALITY_TOPIC:-/map_lidar/quality}"
OUTPUT_POSE_TOPIC="${RC_CONTINUOUS_POSE_OUTPUT_TOPIC:-/robot/pose_continuous}"
OUTPUT_ODOM_TOPIC="${RC_CONTINUOUS_POSE_OUTPUT_ODOM_TOPIC:-/robot/odom_continuous}"
STATUS_TOPIC="${RC_CONTINUOUS_POSE_STATUS_TOPIC:-/robot/pose_continuous/status}"

echo "[continuous_pose] fast_odom=$FAST_ODOM_TOPIC"
echo "[continuous_pose] map_odom=$MAP_ODOM_TOPIC"
echo "[continuous_pose] output_pose=$OUTPUT_POSE_TOPIC output_odom=$OUTPUT_ODOM_TOPIC"
echo "[continuous_pose] status=$STATUS_TOPIC"
echo "[continuous_pose] alpha=${RC_CONTINUOUS_POSE_CORRECTION_ALPHA:-0.05} stamp_match=${RC_CONTINUOUS_POSE_STAMP_MATCH_MODE:-auto}"
echo "[continuous_pose] quality_gate=${RC_CONTINUOUS_POSE_QUALITY_GATE:-1}"

exec ros2 run lidar_localization continuous_robot_pose --ros-args \
  -p fast_odom_topic:="$FAST_ODOM_TOPIC" \
  -p map_odom_topic:="$MAP_ODOM_TOPIC" \
  -p quality_topic:="$QUALITY_TOPIC" \
  -p output_pose_topic:="$OUTPUT_POSE_TOPIC" \
  -p output_odom_topic:="$OUTPUT_ODOM_TOPIC" \
  -p status_topic:="$STATUS_TOPIC" \
  -p output_frame_id:="${RC_CONTINUOUS_POSE_FRAME_ID:-robot_origin}" \
  -p child_frame_id:="${RC_CONTINUOUS_POSE_CHILD_FRAME_ID:-robot_base}" \
  -p output_stamp_mode:="${RC_CONTINUOUS_POSE_STAMP_MODE:-now}" \
  -p lidar_to_base_xyz:="${RC_LIDAR_TO_BASE_XYZ:-[0.237488, -0.272804, -0.483]}" \
  -p lidar_to_base_rpy:="${RC_LIDAR_TO_BASE_RPY:-[0.0, 0.0, -1.5707963267948966]}" \
  -p reference_pose_enabled:="${RC_CONTINUOUS_POSE_REFERENCE_ENABLED:-true}" \
  -p reference_pose_xyz:="${RC_ROBOT_ORIGIN_XYZ:-[-0.260, 0.262, -0.483]}" \
  -p reference_pose_rpy:="${RC_ROBOT_ORIGIN_RPY:-[0.0, 0.0, 0.0]}" \
  -p correction_alpha:="${RC_CONTINUOUS_POSE_CORRECTION_ALPHA:-0.05}" \
  -p odom_buffer_sec:="${RC_CONTINUOUS_POSE_ODOM_BUFFER_SEC:-5.0}" \
  -p max_stamp_delta_sec:="${RC_CONTINUOUS_POSE_MAX_STAMP_DELTA:-0.20}" \
  -p stamp_match_mode:="${RC_CONTINUOUS_POSE_STAMP_MATCH_MODE:-auto}" \
  -p quality_gate_enabled:="${RC_CONTINUOUS_POSE_QUALITY_GATE:-true}" \
  -p quality_timeout_sec:="${RC_CONTINUOUS_POSE_QUALITY_TIMEOUT:-1.0}" \
  -p quality_max_fitness:="${RC_CONTINUOUS_POSE_QUALITY_MAX_FITNESS:-0.20}" \
  -p quality_max_jump_m:="${RC_CONTINUOUS_POSE_QUALITY_MAX_JUMP:-0.08}" \
  -p log_period_sec:="${RC_CONTINUOUS_POSE_LOG_PERIOD:-1.0}"
