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
OVERLAY_ENABLED="${OVERLAY_ENABLED:-true}"
OVERLAY_FRAME_ID="${OVERLAY_FRAME_ID:-map}"
OVERLAY_SCRIPT_PATH="${OVERLAY_SCRIPT_PATH:-${ROBOT_ROOT}/scripts/rviz_status_overlay.py}"

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

overlay_pid=""
cleanup() {
  if [[ -n "${overlay_pid}" ]] && kill -0 "${overlay_pid}" 2>/dev/null; then
    kill "${overlay_pid}" >/dev/null 2>&1 || true
    wait "${overlay_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "${OVERLAY_ENABLED,,}" == "true" ]]; then
  if [[ -f "${OVERLAY_SCRIPT_PATH}" ]]; then
    python3 "${OVERLAY_SCRIPT_PATH}" --robot-ns "${ROBOT_NS}" --frame-id "${OVERLAY_FRAME_ID}" &
    overlay_pid="$!"
    echo "Started RViz overlay helper (pid=${overlay_pid})"
  else
    echo "WARN: overlay script not found: ${OVERLAY_SCRIPT_PATH}"
  fi
fi

ros2 launch office_robot_bringup nav_debug_rviz.launch.py robot_ns:="${ROBOT_NS}" "$@"
