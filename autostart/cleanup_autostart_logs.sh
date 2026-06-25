#!/usr/bin/env bash
set -euo pipefail

LOG_ROOT="${1:-${RC_LIDAR_LOG_ROOT:-${RC_LIDAR_WORKSPACE:-$(pwd)}/logs/autostart}}"
RETENTION_DAYS="${RC_AUTOSTART_LOG_RETENTION_DAYS:-30}"
KEEP_RECENT="${RC_AUTOSTART_LOG_KEEP_RECENT:-10}"
DRY_RUN="${RC_AUTOSTART_LOG_CLEANUP_DRY_RUN:-0}"

if [[ ! "$RETENTION_DAYS" =~ ^[0-9]+$ ]] || [ "$RETENTION_DAYS" -le 0 ]; then
  echo "[log_cleanup] disabled: RC_AUTOSTART_LOG_RETENTION_DAYS=$RETENTION_DAYS"
  exit 0
fi

if [[ ! "$KEEP_RECENT" =~ ^[0-9]+$ ]]; then
  KEEP_RECENT=10
fi

if [ ! -d "$LOG_ROOT" ]; then
  echo "[log_cleanup] skipped: log root does not exist: $LOG_ROOT"
  exit 0
fi

declare -A KEEP_PATHS=()

remember_symlink_target() {
  local link="$1"
  local target
  if [ -L "$link" ]; then
    target="$(readlink -f "$link" 2>/dev/null || true)"
    if [ -n "$target" ] && [ -d "$target" ]; then
      KEEP_PATHS["$target"]=1
    fi
  fi
}

remember_symlink_target "$LOG_ROOT/latest"
remember_symlink_target "$LOG_ROOT/latest_dry_run"
remember_symlink_target "$LOG_ROOT/latest_restart"

while IFS= read -r dir; do
  [ -n "$dir" ] && KEEP_PATHS["$dir"]=1
done < <(
  find "$LOG_ROOT" -maxdepth 1 -type d -name 'run_*' -printf '%T@ %p\n' 2>/dev/null \
    | sort -nr \
    | head -n "$KEEP_RECENT" \
    | cut -d' ' -f2-
)

echo "[log_cleanup] root=$LOG_ROOT retention_days=$RETENTION_DAYS keep_recent=$KEEP_RECENT dry_run=$DRY_RUN"

deleted_count=0
while IFS= read -r dir; do
  [ -n "$dir" ] || continue
  abs_dir="$(readlink -f "$dir" 2>/dev/null || true)"
  [ -n "$abs_dir" ] || continue
  if [ "${KEEP_PATHS[$abs_dir]+set}" = "set" ]; then
    echo "[log_cleanup] keep: $abs_dir"
    continue
  fi
  if [ "$DRY_RUN" = "1" ]; then
    echo "[log_cleanup] would delete: $abs_dir"
  else
    echo "[log_cleanup] delete: $abs_dir"
    rm -rf -- "$abs_dir"
  fi
  deleted_count=$((deleted_count + 1))
done < <(
  find "$LOG_ROOT" -maxdepth 1 -type d -name 'run_*' -mtime +"$RETENTION_DAYS" -print 2>/dev/null
)

echo "[log_cleanup] done: candidates_deleted=$deleted_count"
