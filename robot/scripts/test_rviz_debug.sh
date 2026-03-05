#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# One-shot command for operators:
#   ./robot/scripts/test_rviz_debug.sh
exec "${SCRIPT_DIR}/run_rviz_nav_debug.sh" "$@"
