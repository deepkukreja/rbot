# SPDX-License-Identifier: Apache-2.0
# Source from a shell that has the guarded rbot workspace.
_rbot_coupled_activate() {
  local runtime
  if [[ $# != 1 || ! "${ROS_DOMAIN_ID:-}" =~ ^([1-9]|[1-9][0-9]|10[01])$ || -z "${GZ_PARTITION:-}" ]]; then
    printf '%s\n' 'Set a free ROS_DOMAIN_ID (1–101), a unique GZ_PARTITION, and pass the runtime directory.' >&2
    return 2
  fi
  runtime="$(cd -- "$1" && pwd -P)" || return 2
  if [[ ! -f "$runtime/install/local_setup.bash" ||
        ! -f "$runtime/install/diff_drive_controller/lib/libdiff_drive_controller.so" ||
        ! -f "$runtime/install/nav2_mppi_controller/lib/libmppi_controller.so" ||
        ! -f "$runtime/profiles/share/rlai_control/config/controllers.yaml" ||
        ! -f "$runtime/profiles/share/rlai_navigation/config/nav2_params.yaml" ]]; then
    printf '%s\n' 'Build the controller overlay and generate both parameter profiles first.' >&2
    return 2
  fi
  source "$runtime/install/local_setup.bash" || return 2
  export AMENT_PREFIX_PATH="$runtime/profiles${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf '%s\n' 'Source activate.bash from the launch shell.' >&2
  exit 2
fi
if _rbot_coupled_activate "$@"; then
  unset -f _rbot_coupled_activate
else
  unset -f _rbot_coupled_activate
  return 2
fi
