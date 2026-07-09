#!/usr/bin/env bash

lidar_ui_config_path() {
  if [ -n "${RC_LIDAR_UI_CONFIG:-}" ]; then
    printf '%s\n' "$RC_LIDAR_UI_CONFIG"
  elif [ -n "${XDG_CONFIG_HOME:-}" ]; then
    printf '%s\n' "$XDG_CONFIG_HOME/rc_lidar_r2_v2/ui_config.env"
  else
    printf '%s\n' "${HOME:-/tmp}/.config/rc_lidar_r2_v2/ui_config.env"
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

apply_lidar_ui_boot_map_override() {
  if [ -n "${RC_UI_BOOT_MAP:-}" ]; then
    RC_LIDAR_MAP_PATH="$RC_UI_BOOT_MAP"
    RC_LOCALIZATION_MAP_PATH="$RC_UI_BOOT_MAP"
    export RC_LIDAR_MAP_PATH
    export RC_LOCALIZATION_MAP_PATH
  fi
}
