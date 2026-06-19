#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOSTART_DIR="$HOME/.config/autostart"
DESKTOP_FILE="$AUTOSTART_DIR/rc_lidar_autostart.desktop"

mkdir -p "$AUTOSTART_DIR"
install -m 0644 "$DIR/rc_lidar_autostart.desktop" "$DESKTOP_FILE"

echo "[desktop-autostart] installed: $DESKTOP_FILE"
echo "[desktop-autostart] it will run after the orangepi graphical desktop session starts."
echo "[desktop-autostart] log: /home/orangepi/RC_LIDAR_NOW!!!/logs/autostart/xdg_autostart.log"
