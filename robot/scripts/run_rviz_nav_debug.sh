#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source_safe() {
  local setup_file="$1"
  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
}

RUNTIME_ENV_FILE="${RUNTIME_ENV_FILE:-/etc/robot_runtime.env}"
if [[ -f "${RUNTIME_ENV_FILE}" ]]; then
  set -a
  # shellcheck source=/dev/null
  source "${RUNTIME_ENV_FILE}"
  set +a
fi

ROBOT_NS="${ROBOT_NS:-robot01}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
ROBOT_WS_SETUP_PATH="${ROBOT_WS_SETUP_PATH:-${ROBOT_ROOT}/jazzy_ws/install/setup.bash}"
EXTRA_SETUP_PATH="${EXTRA_SETUP_PATH:-}"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source_safe /opt/ros/jazzy/setup.bash
fi
if [[ -f "${ROBOT_WS_SETUP_PATH}" ]]; then
  source_safe "${ROBOT_WS_SETUP_PATH}"
fi
if [[ -n "${EXTRA_SETUP_PATH}" ]] && [[ -f "${EXTRA_SETUP_PATH}" ]]; then
  source_safe "${EXTRA_SETUP_PATH}"
fi

export ROS_DOMAIN_ID

if [[ -z "${DISPLAY:-}" ]]; then
  echo "WARN: DISPLAY is empty. RViz GUI will not open without X/Wayland session."
fi

echo "Launching RViz debug from PC (robot_ns=${ROBOT_NS}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
exec ros2 launch office_robot_bringup nav_debug_rviz.launch.py robot_ns:="${ROBOT_NS}" "$@"
