#!/usr/bin/env bash
set -euo pipefail

IFACE="${LIVOX_NET_IFACE:-eth0}"
HOST_CIDR="${LIVOX_HOST_CIDR:-192.168.1.5/24}"
LIDAR_IP="${LIVOX_LIDAR_IP:-192.168.1.124}"
ALLOW_SUDO="${LIVOX_NET_ALLOW_SUDO:-0}"

echo "[livox-net] interface: $IFACE"
echo "[livox-net] host cidr: $HOST_CIDR"
echo "[livox-net] lidar ip:   $LIDAR_IP"
echo

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "[livox-net] ERROR: interface $IFACE not found."
  ip -brief link
  exit 1
fi

echo "[livox-net] before:"
ip -brief addr show "$IFACE"
ip route get 8.8.8.8 2>/dev/null || true
echo

HOST_IP="${HOST_CIDR%/*}"
echo "[livox-net] checking for duplicate $HOST_IP assignments..."
ip -brief addr | awk -v host_ip="$HOST_IP" '$0 ~ host_ip { print "  " $0 }'
echo

configure_with_nmcli() {
  if ! command -v nmcli >/dev/null 2>&1; then
    return 1
  fi

  if ! systemctl is-active NetworkManager >/dev/null 2>&1; then
    return 1
  fi

  local conn
  conn="$(nmcli -g GENERAL.CONNECTION dev show "$IFACE" 2>/dev/null | head -n 1 || true)"
  if [ -z "$conn" ] || [ "$conn" = "--" ]; then
    conn="$(nmcli -t -f NAME,TYPE,DEVICE con show 2>/dev/null | awk -F: -v iface="$IFACE" '$2 == "802-3-ethernet" && $3 == iface { print $1; exit }')"
  fi

  if [ -z "$conn" ]; then
    conn="RC_LIVOX_${IFACE}"
    echo "[livox-net] creating NetworkManager connection $conn ..."
    nmcli con add type ethernet ifname "$IFACE" con-name "$conn" \
      ipv4.method manual ipv4.addresses "$HOST_CIDR" ipv4.never-default yes \
      connection.autoconnect yes >/dev/null
  else
    echo "[livox-net] using NetworkManager connection: $conn"
    nmcli con mod "$conn" \
      connection.interface-name "$IFACE" connection.autoconnect yes \
      ipv4.method manual ipv4.addresses "$HOST_CIDR" ipv4.gateway "" \
      ipv4.never-default yes
  fi

  if ! ip -brief addr show "$IFACE" | grep -q "${HOST_CIDR%/*}/"; then
    echo "[livox-net] activating $conn on $IFACE ..."
    nmcli con up "$conn" ifname "$IFACE" >/dev/null
  fi
}

if ! ip -brief addr show "$IFACE" | grep -q "${HOST_CIDR%/*}/"; then
  if configure_with_nmcli; then
    echo "[livox-net] NetworkManager configured $IFACE for $HOST_CIDR."
  else
    echo "[livox-net] WARN: NetworkManager configuration failed or is unavailable."
  fi
fi

if ip -brief addr show "$IFACE" | grep -q "${HOST_CIDR%/*}/"; then
  echo "[livox-net] $HOST_CIDR already exists on $IFACE."
elif [ "$ALLOW_SUDO" = "1" ]; then
  if ip -brief link show "$IFACE" | grep -q '<[^>]*UP'; then
    echo "[livox-net] $IFACE is already UP."
  else
    echo "[livox-net] bringing $IFACE up ..."
    sudo ip link set "$IFACE" up
  fi

  echo "[livox-net] adding secondary address $HOST_CIDR to $IFACE ..."
  sudo ip addr add "$HOST_CIDR" dev "$IFACE"
else
  echo "[livox-net] ERROR: $HOST_CIDR is missing on $IFACE and sudo fallback is disabled."
  echo "[livox-net] Run once: ./autostart/install_one_click_network.sh"
  echo "[livox-net] Or temporarily allow sudo fallback: LIVOX_NET_ALLOW_SUDO=1 $0"
  exit 3
fi

echo
echo "[livox-net] after:"
ip -brief addr show "$IFACE"
ip route get 8.8.8.8 2>/dev/null || true
ip route get "$LIDAR_IP" 2>/dev/null || true
echo

echo "[livox-net] probing route-selected interface..."
ip route get "$LIDAR_IP" 2>/dev/null || true
echo

echo "[livox-net] probing lidar with ARP..."
if arping -I "$IFACE" -c 3 "$LIDAR_IP"; then
  echo "[livox-net] OK: lidar replied on $LIDAR_IP."
else
  echo "[livox-net] WARN: no ARP reply from $LIDAR_IP."
  echo "[livox-net] This usually means the lidar is not on this cable/interface,"
  echo "[livox-net] is not powered, or its real IP is different from $LIDAR_IP."
  exit 2
fi
