#!/usr/bin/env bash
set -euo pipefail

if ! command -v v4l2-ctl >/dev/null 2>&1; then
  echo "/dev/video0"
  exit 0
fi

for dev in /dev/video*; do
  [ -e "$dev" ] || continue
  if ! v4l2-ctl -d "$dev" --all >/dev/null 2>&1; then
    continue
  fi
  if v4l2-ctl -d "$dev" --all 2>/dev/null | grep -qi "Video Capture"; then
    echo "$dev"
    exit 0
  fi
done

echo "/dev/video0"
exit 0
