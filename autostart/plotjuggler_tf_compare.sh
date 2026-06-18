#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

LAYOUT_FILE="${PLOTJUGGLER_TF_LAYOUT:-$RC_LIDAR_WORKSPACE/src/fastlio_world_init/config/tf_compare_plotjuggler.xml}"
WAIT_SECONDS="${PLOTJUGGLER_WAIT_SECONDS:-90}"
BUFFER_SIZE="${PLOTJUGGLER_BUFFER_SIZE:-300}"

TOPICS=(
  "/tf_compare/robot_init_to_base_link"
  "/tf_compare/robot_init_to_base_link_highrate"
  "/tf_compare/lidar_world_to_body"
)

if [ ! -f "$LAYOUT_FILE" ]; then
  echo "[plotjuggler] ERROR: layout file not found: $LAYOUT_FILE"
  exit 2
fi

wait_for_topics() {
  local deadline=$((SECONDS + WAIT_SECONDS))
  local topics
  local missing

  while [ "$SECONDS" -lt "$deadline" ]; do
    topics="$(ros2 topic list 2>/dev/null || true)"
    missing=()

    for topic in "${TOPICS[@]}"; do
      if ! grep -Fxq "$topic" <<< "$topics"; then
        missing+=("$topic")
      fi
    done

    if [ "${#missing[@]}" -eq 0 ]; then
      echo "[plotjuggler] All tf_compare topics are available."
      return 0
    fi

    echo "[plotjuggler] Waiting for topics: ${missing[*]}"
    sleep 1
  done

  echo "[plotjuggler] WARN: timeout waiting for tf_compare topics; starting PlotJuggler anyway."
  return 0
}

auto_confirm_plotjuggler_dialogs() {
  if ! command -v xdotool >/dev/null 2>&1; then
    echo "[plotjuggler] WARN: xdotool not found; PlotJuggler dialogs must be confirmed manually."
    echo "[plotjuggler] Install it with: sudo apt install -y xdotool"
    return 0
  fi

  (
    local deadline=$((SECONDS + 75))
    local patterns=(
      "Warning"
      "Start Streaming?"
      "Topic Selector"
      "Select ROS topics"
    )
    local pattern
    local windows
    local window

    while [ "$SECONDS" -lt "$deadline" ]; do
      for pattern in "${patterns[@]}"; do
        windows="$(xdotool search --name "$pattern" 2>/dev/null || true)"
        for window in $windows; do
          echo "[plotjuggler] Auto-confirming dialog '$pattern' window=$window"
          xdotool windowactivate --sync "$window" key Return 2>/dev/null || true
        done
      done
      sleep 0.5
    done
  ) &
}

wait_for_topics
auto_confirm_plotjuggler_dialogs

echo "[plotjuggler] Starting PlotJuggler with layout: $LAYOUT_FILE"

exec ros2 run plotjuggler plotjuggler \
  --nosplash \
  --layout "$LAYOUT_FILE" \
  --start_streamer DataStreamROS2 \
  --buffer_size "$BUFFER_SIZE" \
  --window_title "TF Compare"
