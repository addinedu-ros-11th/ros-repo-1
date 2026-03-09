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
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/cmd_vel}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
STOP_BURST_SEC="${STOP_BURST_SEC:-2}"
STOP_BURST_HZ="${STOP_BURST_HZ:-15}"
SPIN_SEC="${SPIN_SEC:-0}"
SPIN_DEG="${SPIN_DEG:-0}"
SPIN_ANGULAR_Z="${SPIN_ANGULAR_Z:-0.0}"
RELOCALIZE="${RELOCALIZE:-false}"
STATUS_TOPIC="${STATUS_TOPIC:-/${ROBOT_NS}/status}"
AMCL_POSE_TOPIC="${AMCL_POSE_TOPIC:-/${ROBOT_NS}/amcl_pose}"
RELOCALIZE_SERVICE="${RELOCALIZE_SERVICE:-/${ROBOT_NS}/reinitialize_global_localization}"
NOMOTION_SERVICE="${NOMOTION_SERVICE:-/${ROBOT_NS}/request_nomotion_update}"
TURN_SCRIPT_PATH="${TURN_SCRIPT_PATH:-${SCRIPT_DIR}/turn_in_place.py}"

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

echo "[localization-reset] robot_ns=${ROBOT_NS} ros_domain_id=${ROS_DOMAIN_ID}"
echo "[localization-reset] stop cmd_vel burst on ${CMD_VEL_TOPIC}"
timeout "${STOP_BURST_SEC}" ros2 topic pub "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  -r "${STOP_BURST_HZ}" >/dev/null 2>&1 || true

if [[ "${RELOCALIZE,,}" == "true" ]]; then
  echo "[localization-reset] call ${RELOCALIZE_SERVICE}"
  ros2 service call "${RELOCALIZE_SERVICE}" std_srvs/srv/Empty "{}" || true
else
  echo "[localization-reset] skip ${RELOCALIZE_SERVICE} (preserve current / manual initial pose)"
fi

echo "[localization-reset] call ${NOMOTION_SERVICE}"
ros2 service call "${NOMOTION_SERVICE}" std_srvs/srv/Empty "{}" || true

if [[ "${SPIN_DEG}" != "0" ]] && [[ "${SPIN_ANGULAR_Z}" != "0" ]]; then
  echo "[localization-reset] odom-based spin (${SPIN_DEG}deg, angular_z=${SPIN_ANGULAR_Z})"
  python3 "${TURN_SCRIPT_PATH}" \
    --odom-topic "${ODOM_TOPIC}" \
    --cmd-vel-topic "${CMD_VEL_TOPIC}" \
    --angle-deg "${SPIN_DEG}" \
    --angular-speed "${SPIN_ANGULAR_Z}" || true
elif [[ "${SPIN_SEC}" != "0" ]] && [[ "${SPIN_ANGULAR_Z}" != "0" ]]; then
  echo "[localization-reset] legacy time-based spin (${SPIN_SEC}s, angular_z=${SPIN_ANGULAR_Z})"
  timeout "${SPIN_SEC}" ros2 topic pub "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${SPIN_ANGULAR_Z}}}" \
    -r 10 >/dev/null 2>&1 || true
  timeout "${STOP_BURST_SEC}" ros2 topic pub "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    -r "${STOP_BURST_HZ}" >/dev/null 2>&1 || true
fi

echo "[localization-reset] latest AMCL pose"
timeout 5 ros2 topic echo "${AMCL_POSE_TOPIC}" --once || true

echo "[localization-reset] latest status"
timeout 5 ros2 topic echo "${STATUS_TOPIC}" --once || true

cat <<EOF
[localization-reset] next steps
1. If RViz is not open:
   ROBOT_NS=${ROBOT_NS} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ./robot/scripts/run_rviz_nav_debug.sh
2. If you already used '2D Pose Estimate', this helper preserved that pose unless RELOCALIZE=true.
3. Confirm red LaserScan aligns with black map walls before sending GOTO.
4. If you want one full refinement spin after a manual pose estimate:
   ROBOT_NS=${ROBOT_NS} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} SPIN_DEG=360 SPIN_ANGULAR_Z=0.35 ./robot/scripts/reset_localization.sh
5. If the robot is completely lost and you want a fresh global relocalization first:
   ROBOT_NS=${ROBOT_NS} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} RELOCALIZE=true ./robot/scripts/reset_localization.sh
   Then set '2D Pose Estimate' again in RViz.
EOF
