#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$DIR/.." && pwd)"
SERVICE_NAME="rc-lidar-autostart.service"
SOURCE_SERVICE="$DIR/$SERVICE_NAME"
USER_SYSTEMD_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
TARGET_SERVICE="$USER_SYSTEMD_DIR/$SERVICE_NAME"

if [ ! -f "$SOURCE_SERVICE" ]; then
  echo "[systemd-autostart] missing service template: $SOURCE_SERVICE" >&2
  exit 1
fi

mkdir -p "$USER_SYSTEMD_DIR"
install -m 0644 "$SOURCE_SERVICE" "$TARGET_SERVICE"

systemctl --user daemon-reload
systemctl --user enable "$SERVICE_NAME"

echo "[systemd-autostart] installed: $TARGET_SERVICE"
echo "[systemd-autostart] enabled user unit: $SERVICE_NAME"
echo "[systemd-autostart] it will run boot_start.sh on the next user-session start."
echo "[systemd-autostart] XDG desktop autostart may still call boot_start.sh; boot_id guarding prevents duplicate real starts."
echo "[systemd-autostart] status:"
systemctl --user --no-pager status "$SERVICE_NAME" || true
