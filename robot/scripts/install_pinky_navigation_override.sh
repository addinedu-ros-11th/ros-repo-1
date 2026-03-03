#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${SERVICE_NAME:-pinky-navigation.service}"
ENV_FILE="${ENV_FILE:-/etc/robot_runtime.env}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMPLATE_PATH="${TEMPLATE_PATH:-${REPO_ROOT}/systemd/pinky-navigation.override.conf.example}"

OVERRIDE_DIR="/etc/systemd/system/${SERVICE_NAME}.d"
OVERRIDE_PATH="${OVERRIDE_DIR}/override.conf"

if (( EUID != 0 )); then
  echo "This script must run as root."
  echo "Example: sudo $0"
  exit 1
fi

if [[ ! -f "${TEMPLATE_PATH}" ]]; then
  echo "Template not found: ${TEMPLATE_PATH}"
  exit 1
fi

install -d "${OVERRIDE_DIR}"
install -m 0644 "${TEMPLATE_PATH}" "${OVERRIDE_PATH}"
systemctl daemon-reload

echo "Installed: ${OVERRIDE_PATH}"

if [[ -f "${ENV_FILE}" ]]; then
  if ! grep -Eq '^NAV2_PARAMS_FILE=' "${ENV_FILE}"; then
    echo "WARN: NAV2_PARAMS_FILE is missing in ${ENV_FILE}"
  fi
  if ! grep -Eq '^MAP_PATH=' "${ENV_FILE}"; then
    echo "WARN: MAP_PATH is missing in ${ENV_FILE}"
  fi
else
  echo "WARN: runtime env file not found: ${ENV_FILE}"
fi

echo
echo "Next commands:"
echo "  sudo systemctl restart ${SERVICE_NAME}"
echo "  systemctl --no-pager --full status ${SERVICE_NAME}"
echo "  systemctl cat ${SERVICE_NAME} | sed -n '1,160p'"
echo "  /home/pinky/ros-repo-1/robot/scripts/nav2_runtime_audit.sh"
