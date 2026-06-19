#!/usr/bin/env bash
set -euo pipefail

CAN_INTERFACE="${CAN_INTERFACE:-}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
CAN_RESTART_MS="${CAN_RESTART_MS:-100}"
CAN_SETUP_ENV_FILE="${CAN_SETUP_ENV_FILE:-}"
CHECK_ONLY=0

usage() {
  cat <<EOF
Usage: CAN_INTERFACE=can0 CAN_BITRATE=1000000 $0 [--check-only]

Checks and, unless --check-only is used, configures the SocketCAN interface.
If CAN_INTERFACE is not set, the first detected can* or slcan* interface is used.
EOF
}

for arg in "$@"; do
  case "$arg" in
    --check-only)
      CHECK_ONLY=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[can_setup] ERROR: unknown argument: $arg"
      usage
      exit 64
      ;;
  esac
done

log() {
  echo "[can_setup] $*"
}

is_iface_up() {
  ip link show "$CAN_INTERFACE" 2>/dev/null | grep -q '<[^>]*UP'
}

detect_can_interface() {
  local iface

  iface="$(ip -o link show type can 2>/dev/null | sed -n 's/^[0-9]\+: \([^:@]\+\).*/\1/p' | head -n 1)"
  if [ -n "$iface" ]; then
    echo "$iface"
    return 0
  fi

  for path in /sys/class/net/can* /sys/class/net/slcan*; do
    [ -e "$path" ] || continue
    basename "$path"
    return 0
  done

  return 1
}

write_env_file() {
  [ -n "$CAN_SETUP_ENV_FILE" ] || return 0

  {
    echo "CAN_INTERFACE='$CAN_INTERFACE'"
    echo "CAN_BITRATE='$CAN_BITRATE'"
    echo "CAN_RESTART_MS='$CAN_RESTART_MS'"
  } > "$CAN_SETUP_ENV_FILE"
}

print_usb_candidates() {
  log "USB-CAN candidates from lsusb:"
  if command -v lsusb >/dev/null 2>&1; then
    lsusb | grep -Ei 'can|peak|pcan|xcan|0c72|zlg|kvaser|gs_usb' || \
      echo "[can_setup]   none found"
  else
    echo "[can_setup]   lsusb not installed"
  fi
}

print_recent_kernel_can_logs() {
  log "Recent kernel USB/CAN messages:"
  if command -v dmesg >/dev/null 2>&1; then
    dmesg --ctime 2>/dev/null | \
      grep -Ei 'can|peak|pcan|xcan|0c72|usb.*disconnect|error -71|error -62' | \
      tail -n 40 || echo "[can_setup]   none found or dmesg unavailable"
  else
    echo "[can_setup]   dmesg not available"
  fi
}

print_link_snapshot() {
  log "Network link snapshot:"
  ip -brief link || true
  log "SocketCAN link snapshot:"
  ip -details link show type can 2>/dev/null || true
  if [ -n "$CAN_INTERFACE" ] && ip link show "$CAN_INTERFACE" >/dev/null 2>&1; then
    log "$CAN_INTERFACE details:"
    ip -details -statistics link show "$CAN_INTERFACE" || true
  fi
}

print_missing_help() {
  if [ -n "$CAN_INTERFACE" ]; then
    log "ERROR: requested CAN interface $CAN_INTERFACE does not exist."
  else
    log "ERROR: no SocketCAN interface was detected."
  fi
  log "This means rc_serial_driver cannot open a SocketCAN socket, so no CAN frames can be sent."
  log "Replug the USB-CAN adapter, try another USB port or a powered USB hub, then check:"
  log "  lsusb"
  log "  ip -details link show type can"
  log "  ip -brief link"
}

if ! command -v ip >/dev/null 2>&1; then
  log "ERROR: ip command not found."
  exit 127
fi

print_link_snapshot
print_usb_candidates

if [ -z "$CAN_INTERFACE" ]; then
  if CAN_INTERFACE="$(detect_can_interface)"; then
    export CAN_INTERFACE
    log "Auto-detected CAN interface: $CAN_INTERFACE"
  fi
fi

if [ -z "$CAN_INTERFACE" ] || ! ip link show "$CAN_INTERFACE" >/dev/null 2>&1; then
  print_missing_help
  print_recent_kernel_can_logs
  exit 2
fi

write_env_file

if [ "$CHECK_ONLY" = "1" ]; then
  if is_iface_up; then
    log "$CAN_INTERFACE exists and has the UP flag."
  else
    log "$CAN_INTERFACE exists but is not UP yet."
  fi
  exit 0
fi

if sudo -n true >/dev/null 2>&1; then
  log "Configuring $CAN_INTERFACE bitrate=$CAN_BITRATE restart-ms=$CAN_RESTART_MS ..."
  sudo ip link set "$CAN_INTERFACE" down || true
  sudo ip link set "$CAN_INTERFACE" type can bitrate "$CAN_BITRATE" restart-ms "$CAN_RESTART_MS"
  sudo ip link set "$CAN_INTERFACE" up
else
  if is_iface_up; then
    log "WARN: sudo is not available without a password; $CAN_INTERFACE is already UP, continuing."
    log "WARN: bitrate was not re-applied. Expected bitrate is $CAN_BITRATE."
  else
    log "ERROR: $CAN_INTERFACE exists but is not UP, and sudo is not available without a password."
    log "For passwordless one-click startup, run once: ./autostart/install_one_click_network.sh"
    log "Run these commands manually before starting CAN_SERIAL:"
    log "  sudo ip link set $CAN_INTERFACE down"
    log "  sudo ip link set $CAN_INTERFACE type can bitrate $CAN_BITRATE restart-ms $CAN_RESTART_MS"
    log "  sudo ip link set $CAN_INTERFACE up"
    exit 3
  fi
fi

if ! is_iface_up; then
  log "ERROR: $CAN_INTERFACE still does not have the UP flag after setup."
  ip -details -statistics link show "$CAN_INTERFACE" || true
  exit 4
fi

log "$CAN_INTERFACE is ready. CAN devices often show state UNKNOWN; the important flag is UP."
ip -details -statistics link show "$CAN_INTERFACE" || true
