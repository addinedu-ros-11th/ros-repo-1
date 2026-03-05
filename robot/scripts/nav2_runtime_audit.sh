#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${SERVICE_NAME:-pinky-navigation.service}"
ROBOT_NS="${ROBOT_NS:-robot01}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
PARAM_TIMEOUT_SEC="${PARAM_TIMEOUT_SEC:-2}"
SKIP_PARAM_WHEN_SERVICE_DOWN="${SKIP_PARAM_WHEN_SERVICE_DOWN:-true}"
RUNTIME_ENV_FILE="${RUNTIME_ENV_FILE:-/etc/robot_runtime.env}"

PINKY_SETUP_PATH="${PINKY_SETUP_PATH:-/home/pinky/pinky_pro/install/setup.bash}"
ROBOT_WS_SETUP_PATH="${ROBOT_WS_SETUP_PATH:-/home/pinky/ros-repo-1/robot/jazzy_ws/install/setup.bash}"

source_safe() {
  local setup_file="$1"
  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
}

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source_safe /opt/ros/jazzy/setup.bash
fi

if [[ -f "${PINKY_SETUP_PATH}" ]]; then
  source_safe "${PINKY_SETUP_PATH}"
elif [[ -f /home/changpc/pinky_pro/install/setup.bash ]]; then
  source_safe /home/changpc/pinky_pro/install/setup.bash
fi

if [[ -f "${ROBOT_WS_SETUP_PATH}" ]]; then
  source_safe "${ROBOT_WS_SETUP_PATH}"
elif [[ -f /home/changpc/ros-repo-1/robot/jazzy_ws/install/setup.bash ]]; then
  source_safe /home/changpc/ros-repo-1/robot/jazzy_ws/install/setup.bash
fi

export ROS_DOMAIN_ID

normalize() {
  printf "%s" "$1" | tr '[:upper:]' '[:lower:]' | tr -d "[]'\" "
}

extract_value() {
  local raw="$1"
  local value
  value="$(printf "%s\n" "$raw" | sed -n -E 's/^.* value is: //p' | tail -n 1)"
  if [[ -n "${value}" ]]; then
    printf "%s" "${value}"
    return
  fi
  value="$(printf "%s\n" "$raw" | sed -n -E 's/^value: //p' | tail -n 1)"
  if [[ -n "${value}" ]]; then
    printf "%s" "${value}"
    return
  fi
  printf "%s" "$(printf "%s\n" "$raw" | tail -n 1)"
}

declare -A PARAMS=()
declare -a FAILS=()
declare -a WARNS=()
service_looks_down=false

read_param() {
  local label="$1"
  local node="$2"
  local key="$3"
  local out
  if ! out="$(timeout "${PARAM_TIMEOUT_SEC}" ros2 param get "${node}" "${key}" 2>&1)"; then
    PARAMS["${label}"]="__ERROR__"
    local brief
    brief="$(printf "%s\n" "${out}" | tail -n 1)"
    if [[ -z "${brief}" ]]; then
      brief="${out//$'\n'/ | }"
    fi
    WARNS+=("${label}: ${brief}")
    return
  fi
  PARAMS["${label}"]="$(extract_value "${out}")"
}

echo "=== NAV2 RUNTIME AUDIT ==="
echo "timestamp: $(date -Iseconds)"
echo "service: ${SERVICE_NAME}"
echo "robot_ns: ${ROBOT_NS}"
echo "ros_domain_id: ${ROS_DOMAIN_ID}"
echo

echo "## systemctl cat ${SERVICE_NAME}"
if ! systemctl cat "${SERVICE_NAME}"; then
  WARNS+=("systemctl cat failed: ${SERVICE_NAME}")
  service_looks_down=true
fi
echo

echo "## MainPID cmdline"
main_pid="$(systemctl show -p MainPID --value "${SERVICE_NAME}" 2>/dev/null || true)"
if [[ "${main_pid}" =~ ^[0-9]+$ ]] && (( main_pid > 1 )) && [[ -r "/proc/${main_pid}/cmdline" ]]; then
  echo "MainPID=${main_pid}"
  tr '\0' ' ' <"/proc/${main_pid}/cmdline"
  echo
else
  echo "MainPID unavailable (${main_pid:-none})"
  WARNS+=("MainPID unavailable for ${SERVICE_NAME}")
  service_looks_down=true
fi
echo

ns="${ROBOT_NS#/}"
amcl_node="/${ns}/amcl"
local_costmap_node="/${ns}/local_costmap/local_costmap"
global_costmap_node="/${ns}/global_costmap/global_costmap"
controller_node="/${ns}/controller_server"
nav2_params_file="${NAV2_PARAMS_FILE:-}"
if [[ -z "${nav2_params_file}" ]] && [[ -f "${RUNTIME_ENV_FILE}" ]]; then
  nav2_params_file="$(sed -n -E 's/^NAV2_PARAMS_FILE=(.*)$/\1/p' "${RUNTIME_ENV_FILE}" | tail -n 1)"
fi

echo "## params file namespace check"
if [[ -n "${nav2_params_file}" ]]; then
  echo "params_file=${nav2_params_file}"
else
  echo "params_file=__UNSET__"
fi

if [[ -n "${nav2_params_file}" ]] && [[ -f "${nav2_params_file}" ]]; then
  ns_keys="$(
    grep -E '^[[:space:]]*/[^ #][^:]*:' "${nav2_params_file}" \
      | sed -E 's#^[[:space:]]*/([^/]+)/.*#\1#' \
      | sort -u \
      | tr '\n' ' '
  )"
  if [[ -n "${ns_keys// }" ]]; then
    echo "params_namespaces=${ns_keys}"
    if [[ " ${ns_keys} " != *" ${ns} "* ]]; then
      FAILS+=("params file namespace mismatch (ROBOT_NS=${ns}, params namespaces=${ns_keys})")
    fi
  else
    echo "params_namespaces=none (namespace-agnostic keys)"
  fi
else
  WARNS+=("params file missing or unreadable: ${nav2_params_file:-__UNSET__}")
fi
echo

echo "## ros2 param snapshot"
for key in \
  amcl.update_min_d \
  amcl.update_min_a \
  amcl.scan_topic \
  local.global_frame \
  local.rolling_window \
  local.width \
  local.height \
  local.voxel_layer.observation_sources \
  global.global_frame \
  global.obstacle_layer.observation_sources \
  controller.follow_path_desired_linear_vel; do
  PARAMS["${key}"]="__ERROR__"
done

if [[ "$(normalize "${SKIP_PARAM_WHEN_SERVICE_DOWN}")" == "true" ]] && [[ "${service_looks_down}" == "true" ]]; then
  WARNS+=("param checks skipped because service appears down")
else
  read_param "amcl.update_min_d" "${amcl_node}" "update_min_d"
  read_param "amcl.update_min_a" "${amcl_node}" "update_min_a"
  read_param "amcl.scan_topic" "${amcl_node}" "scan_topic"

  read_param "local.global_frame" "${local_costmap_node}" "global_frame"
  read_param "local.rolling_window" "${local_costmap_node}" "rolling_window"
  read_param "local.width" "${local_costmap_node}" "width"
  read_param "local.height" "${local_costmap_node}" "height"
  read_param "local.voxel_layer.observation_sources" "${local_costmap_node}" "voxel_layer.observation_sources"

  read_param "global.global_frame" "${global_costmap_node}" "global_frame"
  read_param "global.obstacle_layer.observation_sources" "${global_costmap_node}" "obstacle_layer.observation_sources"

  read_param "controller.follow_path_desired_linear_vel" "${controller_node}" "FollowPath.desired_linear_vel"
fi

for key in \
  amcl.update_min_d \
  amcl.update_min_a \
  amcl.scan_topic \
  local.global_frame \
  local.rolling_window \
  local.width \
  local.height \
  local.voxel_layer.observation_sources \
  global.global_frame \
  global.obstacle_layer.observation_sources \
  controller.follow_path_desired_linear_vel; do
  printf "  %-40s = %s\n" "${key}" "${PARAMS[$key]:-__MISSING__}"
done
echo

if [[ "${PARAMS[local.rolling_window]:-__ERROR__}" != "__ERROR__" ]]; then
  if [[ "$(normalize "${PARAMS[local.rolling_window]}")" != "true" ]]; then
    FAILS+=("local rolling_window is not true")
  fi
fi

if [[ "${PARAMS[local.global_frame]:-__ERROR__}" != "__ERROR__" ]]; then
  if [[ "$(normalize "${PARAMS[local.global_frame]}")" != "odom" ]]; then
    FAILS+=("local global_frame is not odom")
  fi
fi

if [[ "${PARAMS[global.global_frame]:-__ERROR__}" != "__ERROR__" ]]; then
  if [[ "$(normalize "${PARAMS[global.global_frame]}")" != "map" ]]; then
    FAILS+=("global global_frame is not map")
  fi
fi

if [[ "${PARAMS[local.voxel_layer.observation_sources]:-__ERROR__}" != "__ERROR__" ]]; then
  if [[ "$(normalize "${PARAMS[local.voxel_layer.observation_sources]}")" != *scan* ]]; then
    FAILS+=("local voxel_layer observation_sources does not include scan")
  fi
fi

if [[ "${PARAMS[global.obstacle_layer.observation_sources]:-__ERROR__}" != "__ERROR__" ]]; then
  if [[ "$(normalize "${PARAMS[global.obstacle_layer.observation_sources]}")" != *scan* ]]; then
    FAILS+=("global obstacle_layer observation_sources does not include scan")
  fi
fi

echo "## verdict"
if (( ${#FAILS[@]} > 0 )); then
  echo "STATUS=FAIL"
  for item in "${FAILS[@]}"; do
    echo "  - ${item}"
  done
  if (( ${#WARNS[@]} > 0 )); then
    echo "WARNINGS:"
    for item in "${WARNS[@]}"; do
      echo "  - ${item}"
    done
  fi
  exit 2
fi

if (( ${#WARNS[@]} > 0 )); then
  echo "STATUS=WARN"
  for item in "${WARNS[@]}"; do
    echo "  - ${item}"
  done
  exit 1
fi

echo "STATUS=PASS"
echo "  - local/global observation sources include scan"
echo "  - local rolling_window=true"
echo "  - local/global frame pair is odom/map"
exit 0
