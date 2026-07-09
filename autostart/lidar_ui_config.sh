#!/usr/bin/env bash

lidar_ui_autostart_dir() {
  cd "$(dirname "${BASH_SOURCE[0]}")" && pwd
}

lidar_ui_config_path() {
  if [ -n "${RC_LIDAR_UI_CONFIG:-}" ]; then
    printf '%s\n' "$RC_LIDAR_UI_CONFIG"
  elif [ -n "${XDG_CONFIG_HOME:-}" ]; then
    printf '%s\n' "$XDG_CONFIG_HOME/rc_lidar_r2_v2/ui_config.env"
  else
    printf '%s\n' "${HOME:-/tmp}/.config/rc_lidar_r2_v2/ui_config.env"
  fi
}

lidar_user_defaults_path() {
  if [ -n "${RC_LIDAR_USER_DEFAULTS:-}" ]; then
    printf '%s\n' "$RC_LIDAR_USER_DEFAULTS"
  else
    printf '%s\n' "$(lidar_ui_autostart_dir)/user_defaults.env"
  fi
}

load_lidar_ui_config() {
  RC_LIDAR_UI_CONFIG="$(lidar_ui_config_path)"
  export RC_LIDAR_UI_CONFIG

  if [ -r "$RC_LIDAR_UI_CONFIG" ]; then
    local prior_lidar_map_path="${RC_LIDAR_MAP_PATH-__rc_lidar_unset__}"
    local prior_localization_map_path="${RC_LOCALIZATION_MAP_PATH-__rc_lidar_unset__}"

    # shellcheck disable=SC1090
    source "$RC_LIDAR_UI_CONFIG"

    if [ -z "${RC_UI_BOOT_MAP:-}" ]; then
      if [ "$prior_lidar_map_path" = "__rc_lidar_unset__" ]; then
        unset RC_LIDAR_MAP_PATH
      else
        RC_LIDAR_MAP_PATH="$prior_lidar_map_path"
        export RC_LIDAR_MAP_PATH
      fi
      if [ "$prior_localization_map_path" = "__rc_lidar_unset__" ]; then
        unset RC_LOCALIZATION_MAP_PATH
      else
        RC_LOCALIZATION_MAP_PATH="$prior_localization_map_path"
        export RC_LOCALIZATION_MAP_PATH
      fi
    fi
  fi
}

load_lidar_user_defaults() {
  RC_LIDAR_USER_DEFAULTS="$(lidar_user_defaults_path)"
  export RC_LIDAR_USER_DEFAULTS

  if [ ! -r "$RC_LIDAR_USER_DEFAULTS" ]; then
    return 0
  fi

  local keys=(
    RC_RELOCALIZATION_INITIAL_GUESS
    RC_RELOCALIZATION_ASSUME_MAP_ORIGIN
    RC_RELOCALIZATION_INITIAL_X
    RC_RELOCALIZATION_INITIAL_Y
    RC_RELOCALIZATION_INITIAL_Z
    RC_RELOCALIZATION_INITIAL_ROLL
    RC_RELOCALIZATION_INITIAL_PITCH
    RC_RELOCALIZATION_INITIAL_YAW
  )
  local prior_set=()
  local prior_value=()
  local key
  local idx

  for key in "${keys[@]}"; do
    if [ "${!key+x}" = "x" ]; then
      prior_set+=("1")
      prior_value+=("${!key}")
    else
      prior_set+=("0")
      prior_value+=("")
    fi
  done

  # shellcheck disable=SC1090
  source "$RC_LIDAR_USER_DEFAULTS"

  idx=0
  for key in "${keys[@]}"; do
    if [ "${prior_set[$idx]}" = "1" ]; then
      printf -v "$key" '%s' "${prior_value[$idx]}"
      export "$key"
    elif [ "${!key+x}" = "x" ]; then
      export "$key"
    fi
    idx=$((idx + 1))
  done
}

apply_lidar_ui_boot_map_override() {
  if [ -n "${RC_UI_BOOT_MAP:-}" ]; then
    RC_LIDAR_MAP_PATH="$RC_UI_BOOT_MAP"
    RC_LOCALIZATION_MAP_PATH="$RC_UI_BOOT_MAP"
    export RC_LIDAR_MAP_PATH
    export RC_LOCALIZATION_MAP_PATH
  fi
}
