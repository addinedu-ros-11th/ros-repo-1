#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="$ROOT_DIR/jazzy_ws/src/office_robot_bringup/config/executor.yaml"
BRINGUP_LAUNCH="$ROOT_DIR/jazzy_ws/src/office_robot_bringup/launch/bringup.launch.py"
EXECUTOR_LAUNCH="$ROOT_DIR/jazzy_ws/src/office_robot_executor/launch/executor.launch.py"
KEY="dynamic_nav_profile_enabled"

usage() {
  cat <<'EOF'
Usage:
  ./robot/scripts/toggle_dynamic_nav_profile.sh status
  ./robot/scripts/toggle_dynamic_nav_profile.sh on
  ./robot/scripts/toggle_dynamic_nav_profile.sh off

This script edits executor.yaml and launch defaults together.
Apply to runtime separately:
  cd robot/jazzy_ws
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install --packages-select office_robot_executor office_robot_bringup
  sudo systemctl restart office-robot.service
EOF
}

current_value() {
  awk -F': ' -v key="$KEY" '$1 ~ "^[[:space:]]*" key "$" {print $2; exit}' "$CONFIG_FILE"
}

set_yaml_value() {
  local target="$1"
  if ! grep -q "^[[:space:]]*$KEY:" "$CONFIG_FILE"; then
    echo "Missing key '$KEY' in $CONFIG_FILE" >&2
    exit 1
  fi
  perl -0pi -e "s/^([ \t]*\Q$KEY\E:\s*).*\$/\${1}$target/m" "$CONFIG_FILE"
}

set_launch_default() {
  local target="$1"
  local file="$2"
  perl -0pi -e "s/(DeclareLaunchArgument\\(\"$KEY\", default_value=\")(?:true|false)(\"\\))/\${1}$target\${2}/g" "$file"
}

set_value() {
  local target="$1"
  set_yaml_value "$target"
  set_launch_default "$target" "$BRINGUP_LAUNCH"
  set_launch_default "$target" "$EXECUTOR_LAUNCH"
}

cmd="${1:-status}"
case "$cmd" in
  status)
    echo "$KEY=$(current_value)"
    ;;
  on)
    set_value "true"
    echo "$KEY=$(current_value)"
    ;;
  off)
    set_value "false"
    echo "$KEY=$(current_value)"
    ;;
  *)
    usage >&2
    exit 1
    ;;
esac
