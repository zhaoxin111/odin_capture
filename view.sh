#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
bridge_pid=''
if ! ros2 node list 2>/dev/null | grep -Fxq '/odin_jpeg_decoder'; then
  python3 "$ODIN_TOOLS/image_bridge.py" &
  bridge_pid=$!
fi
rviz2 -d "$ODIN_TOOLS/playback.rviz" --ros-args -p use_sim_time:=true &
rviz_pid=$!
cleanup() {
  if [[ -n "$bridge_pid" ]]; then
    kill "$bridge_pid" 2>/dev/null || true
    wait "$bridge_pid" 2>/dev/null || true
  fi
  kill "$rviz_pid" 2>/dev/null || true
  wait "$rviz_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
wait "$rviz_pid"
