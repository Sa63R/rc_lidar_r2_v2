#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${RC_LIDAR_WORKSPACE:-$(cd "$DIR/.." && pwd)}"
AUTOSTART_DIR="$HOME/.config/autostart"
DESKTOP_FILE="$AUTOSTART_DIR/rc_lidar_autostart.desktop"

mkdir -p "$AUTOSTART_DIR"
sed "s#@RC_LIDAR_WORKSPACE@#$WORKSPACE_DIR#g" "$DIR/rc_lidar_autostart.desktop" > "$DESKTOP_FILE"
chmod 0644 "$DESKTOP_FILE"

echo "[desktop-autostart] installed: $DESKTOP_FILE"
echo "[desktop-autostart] it will run after the graphical desktop session starts."
echo "[desktop-autostart] workspace: $WORKSPACE_DIR"
echo "[desktop-autostart] log: $WORKSPACE_DIR/logs/autostart/xdg_autostart.log"
