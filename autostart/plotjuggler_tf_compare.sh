#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common_env.sh"

LAYOUT_FILE="${PLOTJUGGLER_LAYOUT:-${PLOTJUGGLER_TF_LAYOUT:-}}"
WAIT_SECONDS="${PLOTJUGGLER_WAIT_SECONDS:-90}"
BUFFER_SIZE="${PLOTJUGGLER_BUFFER_SIZE:-300}"
PLOTJUGGLER_CONFIG="${PLOTJUGGLER_CONFIG:-$HOME/.config/PlotJuggler/io.plotjuggler.PlotJuggler.ini}"

TOPICS=()

if [ "${RC_LIDAR_MODE:-tracking}" = "relocalization" ]; then
  TOPICS+=(
    "/robot/pose"
    "/map_lidar/quality"
    "/lidar/pose_filter/status"
  )

  if [ "${RC_POSE_FILTER:-1}" != "0" ]; then
    TOPICS+=("/robot/pose_filtered")
  fi

  case "${RC_POSE_FILTER_BETA:-0}" in
    1|true|TRUE|yes|YES|on|ON)
      TOPICS+=(
        "${RC_BETA_FILTERED_POSE_TOPIC:-/robot/pose_filtered_beta}"
        "/lidar/pose_filter_beta/status"
      )
      ;;
  esac
fi

TOPICS+=(
  "/tf_compare/robot_init_to_base_link"
  "/tf_compare/robot_init_to_base_link_highrate"
  "/tf_compare/lidar_world_to_body"
)

join_topics_for_qsettings() {
  local joined=""
  local topic
  for topic in "$@"; do
    if [ -n "$joined" ]; then
      joined+=", "
    fi
    joined+="$topic"
  done
  printf '%s\n' "$joined"
}

update_plotjuggler_default_topics() {
  local topics_csv
  topics_csv="$(join_topics_for_qsettings "${TOPICS[@]}")"
  mkdir -p "$(dirname "$PLOTJUGGLER_CONFIG")"

  PLOTJUGGLER_CONFIG="$PLOTJUGGLER_CONFIG" PLOTJUGGLER_TOPICS="$topics_csv" python3 - <<'PY'
from pathlib import Path
import os

path = Path(os.environ["PLOTJUGGLER_CONFIG"])
topics = os.environ["PLOTJUGGLER_TOPICS"]
lines = path.read_text(encoding="utf-8").splitlines() if path.exists() else []

out = []
in_group = False
seen_group = False
set_key = False

for line in lines:
    stripped = line.strip()
    if stripped.startswith("[") and stripped.endswith("]"):
        if in_group and not set_key:
            out.append(f"default_topics={topics}")
            set_key = True
        in_group = stripped == "[DataStreamROS2]"
        seen_group = seen_group or in_group
        out.append(line)
        continue

    if in_group and stripped.startswith("default_topics="):
        out.append(f"default_topics={topics}")
        set_key = True
    else:
        out.append(line)

if not seen_group:
    if out and out[-1].strip():
        out.append("")
    out.append("[DataStreamROS2]")
    out.append("boolean_strings_to_number=true")
    out.append(f"default_topics={topics}")
    out.append("discard_large_arrays=true")
    out.append("max_array_size=100")
    out.append("remove_suffix_from_strings=true")
    out.append("use_header_stamp=true")
elif in_group and not set_key:
    out.append(f"default_topics={topics}")

path.write_text("\n".join(out) + "\n", encoding="utf-8")
PY

  echo "[plotjuggler] Updated DataStreamROS2 default_topics:"
  echo "[plotjuggler]   $topics_csv"
}

if [ -n "$LAYOUT_FILE" ] && [ ! -f "$LAYOUT_FILE" ]; then
  echo "[plotjuggler] ERROR: layout file not found: $LAYOUT_FILE"
  exit 2
fi

if [ "${PLOTJUGGLER_CONFIG_ONLY:-0}" = "1" ]; then
  update_plotjuggler_default_topics
  exit 0
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
      echo "[plotjuggler] All requested topics are available."
      return 0
    fi

    echo "[plotjuggler] Waiting for topics: ${missing[*]}"
    sleep 1
  done

  echo "[plotjuggler] WARN: timeout waiting for requested topics; starting PlotJuggler anyway."
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
      "ROS2 Topic Subscriber"
    )
    local pattern
    local windows
    local window

    while [ "$SECONDS" -lt "$deadline" ]; do
      for pattern in "${patterns[@]}"; do
        windows="$(xdotool search --name "$pattern" 2>/dev/null || true)"
        for window in $windows; do
          echo "[plotjuggler] Auto-confirming dialog '$pattern' window=$window"
          xdotool windowactivate --sync "$window" 2>/dev/null || true
          case "$pattern" in
            "Topic Selector"|"Select ROS topics"|"ROS2 Topic Subscriber")
              xdotool key --window "$window" alt+o 2>/dev/null || true
              xdotool key --window "$window" Return 2>/dev/null || true
              ;;
            *)
              xdotool key --window "$window" Return 2>/dev/null || true
              ;;
          esac
        done
      done
      sleep 0.5
    done
  ) &
}

wait_for_topics
update_plotjuggler_default_topics
auto_confirm_plotjuggler_dialogs

if [ -n "$LAYOUT_FILE" ]; then
  echo "[plotjuggler] Starting PlotJuggler with layout: $LAYOUT_FILE"
  exec ros2 run plotjuggler plotjuggler \
    --nosplash \
    --layout "$LAYOUT_FILE" \
    --start_streamer DataStreamROS2 \
    --buffer_size "$BUFFER_SIZE" \
    --window_title "TF Compare"
fi

echo "[plotjuggler] Starting PlotJuggler without a forced plot layout."
exec ros2 run plotjuggler plotjuggler \
  --nosplash \
  --start_streamer DataStreamROS2 \
  --buffer_size "$BUFFER_SIZE" \
  --window_title "TF Compare"
