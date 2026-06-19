#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

LIVOX_IFACE="${LIVOX_NET_IFACE:-eth0}"
LIVOX_HOST_CIDR="${LIVOX_HOST_CIDR:-192.168.1.5/24}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
CAN_RESTART_MS="${CAN_RESTART_MS:-100}"

echo "[install] This is a one-time privileged setup for passwordless one-click startup."
echo "[install] LiDAR interface: $LIVOX_IFACE -> $LIVOX_HOST_CIDR"
echo "[install] CAN auto setup: can* bitrate=$CAN_BITRATE restart-ms=$CAN_RESTART_MS"
echo

if command -v nmcli >/dev/null 2>&1 && systemctl is-active NetworkManager >/dev/null 2>&1; then
  conn="$(nmcli -g GENERAL.CONNECTION dev show "$LIVOX_IFACE" 2>/dev/null | head -n 1 || true)"
  if [ -z "$conn" ] || [ "$conn" = "--" ]; then
    conn="RC_LIVOX_${LIVOX_IFACE}"
    echo "[install] Creating NetworkManager connection $conn ..."
    nmcli con add type ethernet ifname "$LIVOX_IFACE" con-name "$conn" \
      ipv4.method manual ipv4.addresses "$LIVOX_HOST_CIDR" ipv4.never-default yes \
      connection.autoconnect yes >/dev/null
  else
    echo "[install] Updating NetworkManager connection $conn ..."
    nmcli con mod "$conn" \
      connection.interface-name "$LIVOX_IFACE" connection.autoconnect yes \
      ipv4.method manual ipv4.addresses "$LIVOX_HOST_CIDR" ipv4.gateway "" \
      ipv4.never-default yes
  fi
  nmcli con up "$conn" ifname "$LIVOX_IFACE" >/dev/null || true
else
  echo "[install] WARN: NetworkManager is unavailable; LiDAR IP may need manual persistent setup."
fi

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

cat > "$tmpdir/rc-lidar-setup-can" <<EOF
#!/usr/bin/env bash
set -e
IFACE="\${1:-can0}"
CAN_BITRATE="\${CAN_BITRATE:-$CAN_BITRATE}"
CAN_RESTART_MS="\${CAN_RESTART_MS:-$CAN_RESTART_MS}"

if ! command -v ip >/dev/null 2>&1; then
  exit 0
fi

ip link show "\$IFACE" >/dev/null 2>&1 || exit 0
ip link set "\$IFACE" down 2>/dev/null || true
ip link set "\$IFACE" type can bitrate "\$CAN_BITRATE" restart-ms "\$CAN_RESTART_MS"
ip link set "\$IFACE" up
EOF

cat > "$tmpdir/99-rc-lidar-can.rules" <<'EOF'
ACTION=="add", SUBSYSTEM=="net", KERNEL=="can*", RUN+="/usr/local/sbin/rc-lidar-setup-can %k"
EOF

echo "[install] Installing udev CAN auto-setup rule. You may be asked for sudo once."
sudo install -m 0755 "$tmpdir/rc-lidar-setup-can" /usr/local/sbin/rc-lidar-setup-can
sudo install -m 0644 "$tmpdir/99-rc-lidar-can.rules" /etc/udev/rules.d/99-rc-lidar-can.rules
sudo udevadm control --reload-rules

if ip -o link show type can 2>/dev/null | grep -q .; then
  for iface in $(ip -o link show type can | sed -n 's/^[0-9]\+: \([^:@]\+\).*/\1/p'); do
    echo "[install] Applying CAN setup to existing $iface ..."
    sudo /usr/local/sbin/rc-lidar-setup-can "$iface"
  done
fi

echo
echo "[install] Done. Future ./autostart/start_all_terminals.sh runs should not need a sudo password for LiDAR/CAN setup."
echo "[install] Current checks:"
"$DIR/setup_livox_net.sh" || true
"$DIR/can_setup.sh" --check-only || true
