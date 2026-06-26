#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${RC_LIDAR_ROOT:-$(cd "$DIR/.." && pwd)}"
CONFIG_FILE="${CANABLE2_CONFIG_FILE:-$REPO_ROOT/src/rc_serial_driver/config/rc_serial_driver.yaml}"
CHECK_ONLY=0
STRICT=0

usage() {
  cat <<EOF
Usage: $0 [--check-only] [--strict]

Diagnose CANable2 serial devices for the R2 CAN FD path. This script does not
configure SocketCAN network interfaces.
EOF
}

for arg in "$@"; do
  case "$arg" in
    --check-only)
      CHECK_ONLY=1
      ;;
    --strict)
      STRICT=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[canable2_preflight] ERROR: unknown argument: $arg"
      usage
      exit 64
      ;;
  esac
done

log() {
  echo "[canable2_preflight] $*"
}

warn() {
  echo "[canable2_preflight] WARN: $*"
}

fail_or_warn() {
  if [ "$STRICT" = "1" ]; then
    log "ERROR: $*"
    exit 2
  fi
  warn "$*"
}

extract_yaml_string() {
  local key="$1"
  sed -n "s/^[[:space:]]*$key:[[:space:]]*\"\{0,1\}\\([^\"#]*\\)\"\{0,1\}.*/\\1/p" "$CONFIG_FILE" | head -n 1 | sed 's/[[:space:]]*$//'
}

log "repo_root: $REPO_ROOT"
log "config: $CONFIG_FILE"

if [ ! -f "$CONFIG_FILE" ]; then
  fail_or_warn "config file not found: $CONFIG_FILE"
  exit 0
fi

serial_device="$(extract_yaml_string serial_device || true)"
serial_prefix="$(extract_yaml_string serial_by_id_prefix || true)"

log "configured serial_device: ${serial_device:-<empty>}"
log "configured serial_by_id_prefix: ${serial_prefix:-<empty>}"
log "user groups: $(id -nG 2>/dev/null || true)"

case " $(id -nG 2>/dev/null || true) " in
  *" dialout "*)
    log "dialout group: present"
    ;;
  *)
    warn "current user is not in dialout; CANable2 serial open may fail"
    ;;
esac

if [ ! -d /dev/serial/by-id ]; then
  fail_or_warn "/dev/serial/by-id does not exist; no stable CANable2 path is visible"
  exit 0
fi

log "CANable2 candidates:"
shopt -s nullglob
candidates=(/dev/serial/by-id/*CANable2*)
if [ "${#candidates[@]}" -eq 0 ]; then
  warn "no /dev/serial/by-id/*CANable2* entries found"
else
  for path in "${candidates[@]}"; do
    target="$(readlink -f "$path" 2>/dev/null || true)"
    perms="$(ls -l "$path" 2>/dev/null || true)"
    log "  $path -> ${target:-unknown}"
    [ -n "$perms" ] && log "    $perms"
  done
fi

if [ -n "$serial_device" ]; then
  configured_path="/dev/serial/by-id/$serial_device"
  if [ -e "$configured_path" ]; then
    log "configured device exists: $configured_path -> $(readlink -f "$configured_path" 2>/dev/null || true)"
  else
    warn "configured device is not present: $configured_path"
  fi
fi

if [ -n "$serial_prefix" ]; then
  prefix_matches=(/dev/serial/by-id/"$serial_prefix"*)
  if [ "${#prefix_matches[@]}" -eq 0 ]; then
    warn "no by-id entries match serial_by_id_prefix '$serial_prefix'"
  elif [ "${#prefix_matches[@]}" -eq 1 ]; then
    log "unique prefix fallback candidate: ${prefix_matches[0]}"
  else
    warn "multiple by-id entries match serial_by_id_prefix '$serial_prefix'; set CAN_DEVICE or an explicit serial_device to avoid ambiguity"
    for path in "${prefix_matches[@]}"; do
      log "  prefix match: $path -> $(readlink -f "$path" 2>/dev/null || true)"
    done
  fi
fi

if [ "$CHECK_ONLY" = "1" ]; then
  log "check-only complete"
fi
