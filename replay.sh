#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
bag="$1"
shift
if [[ -d "$bag/bag" ]]; then bag="$bag/bag"; fi
if ros2 node list | grep -q '/lydros_node'; then
  echo '请先停止实时驱动，再回放相同话题。' >&2
  exit 1
fi
exec ros2 bag play "$bag" --clock "$@"
