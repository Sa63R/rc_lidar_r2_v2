#!/usr/bin/env bash
set -euo pipefail

IFACE="${LIVOX_NET_IFACE:-eth0}"
HOST_CIDR="${LIVOX_HOST_CIDR:-192.168.1.5/24}"
LIDAR_IP="${LIVOX_LIDAR_IP:-192.168.1.124}"

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

sudo ip link set "$IFACE" up

if ip -brief addr show "$IFACE" | grep -q "${HOST_CIDR%/*}/"; then
  echo "[livox-net] $HOST_CIDR already exists on $IFACE."
else
  echo "[livox-net] adding secondary address $HOST_CIDR to $IFACE ..."
  sudo ip addr add "$HOST_CIDR" dev "$IFACE"
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
