#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

CAN_INTERFACE="${CAN_INTERFACE:-}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
CAN_SETUP_SCRIPT="$RC_LIDAR_AUTOSTART/can_setup.sh"
CAN_SETUP_ENV_FILE="${CAN_SETUP_ENV_FILE:-${LOG_DIR:-/tmp}/can_setup.env}"
export CAN_SETUP_ENV_FILE

setup_can_if_possible() {
  if [ ! -x "$CAN_SETUP_SCRIPT" ]; then
    echo "[serial] WARN: CAN setup script not found: $CAN_SETUP_SCRIPT"
    return 0
  fi

  set +e
  "$CAN_SETUP_SCRIPT"
  local setup_rc=$?
  set -e

  if [ -f "$CAN_SETUP_ENV_FILE" ]; then
    # shellcheck disable=SC1090
    source "$CAN_SETUP_ENV_FILE"
    export CAN_INTERFACE
    export CAN_BITRATE
    export CAN_RESTART_MS="${CAN_RESTART_MS:-100}"
  fi

  if [ "$setup_rc" -ne 0 ]; then
    if [ "${RC_CAN_ALLOW_MISSING:-0}" = "1" ]; then
      echo "[serial] WARN: CAN setup failed with exit code $setup_rc, continuing because RC_CAN_ALLOW_MISSING=1."
      return 0
    fi

    echo "[serial] ERROR: CAN setup failed with exit code $setup_rc."
    echo "[serial] ERROR: rc_serial_driver will not be launched because no usable CAN interface was found."
    echo "[serial] HINT: replug USB-CAN, then run: $CAN_SETUP_SCRIPT"
    exit "$setup_rc"
  fi
}

setup_can_if_possible

sleep 3

if [ -z "${RC_CAN_POSE_SOURCE:-}" ]; then
  if [ "${RC_LIDAR_MODE:-tracking}" = "relocalization" ]; then
    RC_CAN_POSE_SOURCE="pose_topic"
  else
    RC_CAN_POSE_SOURCE="tf"
  fi
fi
if [ -z "${RC_CAN_POSE_TOPIC:-}" ]; then
  if [ "$RC_CAN_POSE_SOURCE" != "tf" ] && [ "${RC_LIDAR_MODE:-tracking}" = "relocalization" ]; then
    if [ "${RC_POSE_FILTER:-1}" = "0" ]; then
      RC_CAN_POSE_TOPIC="${RC_RAW_POSE_TOPIC:-/robot/pose}"
    elif [ "${RC_POSE_FILTER_SELECTED:-normal}" = "beta" ]; then
      RC_CAN_POSE_TOPIC="${RC_BETA_FILTERED_POSE_TOPIC:-/robot/pose_filtered_beta}"
    elif [ "${RC_POSE_FILTER_SELECTED:-normal}" = "stable" ]; then
      RC_CAN_POSE_TOPIC="${RC_STABLE_FILTERED_POSE_TOPIC:-/robot/pose_filtered_stable}"
    else
      RC_CAN_POSE_TOPIC="${RC_POSE_FILTER_CAN_TOPIC:-/robot/pose_filtered}"
    fi
  else
    RC_CAN_POSE_TOPIC="/lidar/pose"
  fi
fi
export RC_CAN_POSE_SOURCE
export RC_CAN_POSE_TOPIC

if [ -z "${RC_CAN_USE_YAW_POSE_TOPIC:-}" ]; then
  if [ "${RC_LIDAR_MODE:-tracking}" = "relocalization" ] && [ "$RC_CAN_POSE_SOURCE" = "pose_topic" ]; then
    RC_CAN_USE_YAW_POSE_TOPIC=1
  else
    RC_CAN_USE_YAW_POSE_TOPIC=0
  fi
fi
RC_CAN_YAW_POSE_TOPIC="${RC_CAN_YAW_POSE_TOPIC:-/robot/pose}"
RC_CAN_YAW_POSE_TOPIC_TIMEOUT_SEC="${RC_CAN_YAW_POSE_TOPIC_TIMEOUT_SEC:-0.5}"
RC_CAN_YAW_POSE_TOPIC_STRICT="${RC_CAN_YAW_POSE_TOPIC_STRICT:-1}"
RC_CAN_YAW_IGNORE_READY_GATE="${RC_CAN_YAW_IGNORE_READY_GATE:-1}"
export RC_CAN_USE_YAW_POSE_TOPIC
export RC_CAN_YAW_POSE_TOPIC
export RC_CAN_YAW_POSE_TOPIC_TIMEOUT_SEC
export RC_CAN_YAW_POSE_TOPIC_STRICT
export RC_CAN_YAW_IGNORE_READY_GATE

if [ -z "${RC_CAN_USE_RAW_ODOM_YAW:-}" ]; then
  if [ "${RC_LIDAR_MODE:-tracking}" = "relocalization" ] && [ "$RC_CAN_POSE_SOURCE" = "pose_topic" ]; then
    RC_CAN_USE_RAW_ODOM_YAW=1
  else
    RC_CAN_USE_RAW_ODOM_YAW=0
  fi
fi
RC_CAN_RAW_ODOM_YAW_TOPIC="${RC_CAN_RAW_ODOM_YAW_TOPIC:-/Odometry}"
RC_CAN_RAW_ODOM_YAW_TIMEOUT_SEC="${RC_CAN_RAW_ODOM_YAW_TIMEOUT_SEC:-0.5}"
RC_CAN_RAW_ODOM_YAW_STRICT="${RC_CAN_RAW_ODOM_YAW_STRICT:-1}"
RC_CAN_RAW_ODOM_YAW_OFFSET_RAD="${RC_CAN_RAW_ODOM_YAW_OFFSET_RAD:-1.5707963267948966}"
RC_CAN_RAW_ODOM_YAW_IGNORE_READY_GATE="${RC_CAN_RAW_ODOM_YAW_IGNORE_READY_GATE:-1}"
export RC_CAN_USE_RAW_ODOM_YAW
export RC_CAN_RAW_ODOM_YAW_TOPIC
export RC_CAN_RAW_ODOM_YAW_TIMEOUT_SEC
export RC_CAN_RAW_ODOM_YAW_STRICT
export RC_CAN_RAW_ODOM_YAW_OFFSET_RAD
export RC_CAN_RAW_ODOM_YAW_IGNORE_READY_GATE

if [ -z "${RC_CAN_REQUIRE_READY:-}" ]; then
  if [ "${RC_LIDAR_MODE:-tracking}" = "relocalization" ]; then
    RC_CAN_REQUIRE_READY=1
  else
    RC_CAN_REQUIRE_READY=0
  fi
fi
RC_CAN_READY_QUALITY_TOPIC="${RC_CAN_READY_QUALITY_TOPIC:-/map_lidar/quality}"
RC_CAN_READY_TIMEOUT_SEC="${RC_CAN_READY_TIMEOUT_SEC:-1.0}"
RC_CAN_READY_MAX_FITNESS="${RC_CAN_READY_MAX_FITNESS:-0.05}"
RC_CAN_READY_MIN_ACCEPTS="${RC_CAN_READY_MIN_ACCEPTS:-5}"
RC_CAN_READY_REQUIRE_ACCEPTED="${RC_CAN_READY_REQUIRE_ACCEPTED:-1}"
RC_CAN_READY_REQUIRE_CONVERGED="${RC_CAN_READY_REQUIRE_CONVERGED:-1}"
export RC_CAN_REQUIRE_READY
export RC_CAN_READY_QUALITY_TOPIC
export RC_CAN_READY_TIMEOUT_SEC
export RC_CAN_READY_MAX_FITNESS
export RC_CAN_READY_MIN_ACCEPTS
export RC_CAN_READY_REQUIRE_ACCEPTED
export RC_CAN_READY_REQUIRE_CONVERGED

echo "[serial] Launching rc_serial_driver with CAN_INTERFACE=${CAN_INTERFACE:-unset}, CAN_BITRATE=$CAN_BITRATE"
echo "[serial] pose_source=$RC_CAN_POSE_SOURCE pose_topic=$RC_CAN_POSE_TOPIC"
echo "[serial] yaw_pose_topic_enabled=$RC_CAN_USE_YAW_POSE_TOPIC yaw_pose_topic=$RC_CAN_YAW_POSE_TOPIC yaw_timeout=$RC_CAN_YAW_POSE_TOPIC_TIMEOUT_SEC yaw_strict=$RC_CAN_YAW_POSE_TOPIC_STRICT yaw_ignore_ready_gate=$RC_CAN_YAW_IGNORE_READY_GATE"
echo "[serial] raw_odom_yaw=$RC_CAN_USE_RAW_ODOM_YAW raw_odom_yaw_topic=$RC_CAN_RAW_ODOM_YAW_TOPIC raw_odom_yaw_timeout=$RC_CAN_RAW_ODOM_YAW_TIMEOUT_SEC raw_odom_yaw_strict=$RC_CAN_RAW_ODOM_YAW_STRICT raw_odom_yaw_offset=$RC_CAN_RAW_ODOM_YAW_OFFSET_RAD raw_odom_yaw_ignore_ready_gate=$RC_CAN_RAW_ODOM_YAW_IGNORE_READY_GATE"
echo "[serial] ready_gate=$RC_CAN_REQUIRE_READY quality_topic=$RC_CAN_READY_QUALITY_TOPIC max_fitness=$RC_CAN_READY_MAX_FITNESS min_accepts=$RC_CAN_READY_MIN_ACCEPTS"
if [ "$RC_CAN_POSE_SOURCE" = "pose_topic" ]; then
  echo "[serial] strict pose-topic mode: no TF fallback will be used if $RC_CAN_POSE_TOPIC is stale or missing."
fi
exec ros2 launch rc_serial_driver serial_driver.launch.py
