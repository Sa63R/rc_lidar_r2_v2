#!/usr/bin/env bash

restart_cooldown_active() {
  local now="$1"
  local last="$2"
  local interval="$3"

  [[ "$now" =~ ^[0-9]+$ ]] || return 1
  [[ "$last" =~ ^[0-9]+$ ]] || return 1
  [[ "$interval" =~ ^[0-9]+$ ]] || return 1
  [ "$interval" -gt 0 ] || return 1
  # A lower current uptime means the saved value belongs to an older boot.
  [ "$now" -ge "$last" ] || return 1
  [ $((now - last)) -lt "$interval" ]
}

restart_cooldown_remaining_sec() {
  local now="$1"
  local last="$2"
  local interval="$3"

  restart_cooldown_active "$now" "$last" "$interval" || return 1
  printf '%s\n' $((interval - (now - last)))
}
