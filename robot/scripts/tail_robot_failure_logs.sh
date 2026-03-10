#!/usr/bin/env bash
set -euo pipefail

LINES="${LINES:-80}"

PATTERN="${PATTERN:-Obstacle STOP trigger|Obstacle YIELD_RIGHT trigger|Obstacle SLOW zone|Safety lock enabled|Safety lock disabled|사람 감지로 정지|로봇 감지|Nav2 goal failed|Nav2 goal canceled|STATUS_ABORTED|STATUS_CANCELED|Goal failed|no valid path found|detected collision ahead|Controller patience exceeded|No costmap received|amcl_pose_missing|LOCALIZATION_NOT_READY|ARRIVED_AT_DESTINATION|ACTION_FAILED}"

show_help() {
  cat <<'EOF'
Usage:
  ./robot/scripts/tail_robot_failure_logs.sh [follow]

Behavior:
  - Without args: shows the latest filtered lines from office-robot.service and
    pinky-navigation.service.
  - With "follow": tails both services live and filters for the most useful
    failure/stop/navigation lines.

Environment:
  LINES   Number of recent lines to read before filtering. Default: 80
  PATTERN Custom grep -E pattern for filtering interesting lines.
EOF
}

tail_once() {
  journalctl -u office-robot.service -u pinky-navigation.service -n "${LINES}" --no-pager \
    | grep -E "${PATTERN}" || true
}

tail_follow() {
  journalctl -u office-robot.service -u pinky-navigation.service -f --no-pager \
    | grep --line-buffered -E "${PATTERN}" || true
}

case "${1:-}" in
  "")
    tail_once
    ;;
  follow)
    tail_follow
    ;;
  -h|--help|help)
    show_help
    ;;
  *)
    printf 'Unknown argument: %s\n\n' "${1}" >&2
    show_help >&2
    exit 2
    ;;
esac
