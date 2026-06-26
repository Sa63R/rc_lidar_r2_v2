#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$DIR/.." && pwd)"
SERVICE_NAME="rc-lidar-autostart.service"
SOURCE_SERVICE="$DIR/$SERVICE_NAME"
USER_SYSTEMD_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
TARGET_SERVICE="$USER_SYSTEMD_DIR/$SERVICE_NAME"
XDG_AUTOSTART_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/autostart"
XDG_DISABLED_DIR="$XDG_AUTOSTART_DIR/disabled"

if [ ! -f "$SOURCE_SERVICE" ]; then
  echo "[systemd-autostart] missing service template: $SOURCE_SERVICE" >&2
  exit 1
fi

mkdir -p "$USER_SYSTEMD_DIR"
sed "s#@RC_LIDAR_WORKSPACE@#$WORKSPACE_DIR#g" "$SOURCE_SERVICE" > "$TARGET_SERVICE"
chmod 0644 "$TARGET_SERVICE"

systemctl --user daemon-reload
systemctl --user enable "$SERVICE_NAME"

mkdir -p "$XDG_DISABLED_DIR"
for old_entry in rc_lidar_autostart.desktop rc2_ros_autostart.desktop; do
  old_path="$XDG_AUTOSTART_DIR/$old_entry"
  if [ -e "$old_path" ]; then
    backup_path="$XDG_DISABLED_DIR/${old_entry}.disabled.$(date +%Y%m%d_%H%M%S)"
    mv -f "$old_path" "$backup_path"
    echo "[systemd-autostart] disabled old XDG autostart: $old_path -> $backup_path"
  fi
done

if command -v loginctl >/dev/null 2>&1; then
  if loginctl enable-linger "$(id -un)" >/dev/null 2>&1; then
    echo "[systemd-autostart] enabled linger for user: $(id -un)"
  else
    echo "[systemd-autostart] WARN: could not enable linger for user: $(id -un)" >&2
    echo "[systemd-autostart] WARN: without linger, user services may not start until a user session exists." >&2
  fi
fi

echo "[systemd-autostart] installed: $TARGET_SERVICE"
echo "[systemd-autostart] enabled user unit: $SERVICE_NAME"
echo "[systemd-autostart] it will run boot_start.sh on boot when user linger/session is available."
echo "[systemd-autostart] old LiDAR XDG desktop autostart entries are disabled to keep one boot entry."
echo "[systemd-autostart] status:"
systemctl --user --no-pager status "$SERVICE_NAME" || true
