#!/usr/bin/env bash
set -eo pipefail
if [[ $# -eq 0 || "$1" == --help || "$1" == -h ]]; then
  echo "用法: $0 <采集目录或 bag 目录> [ros2 bag play 参数...]"
  exit 0
fi
source "$(dirname "$0")/env.sh"
bag="$1"
shift
if [[ -d "$bag/bag" ]]; then bag="$bag/bag"; fi
if [[ ! -f "$bag/metadata.yaml" ]]; then
  echo "未找到 bag 元数据: $bag/metadata.yaml" >&2
  exit 1
fi
if ros2 node list | grep -q '/lydros_node'; then
  echo '请先停止实时驱动，再回放相同话题。' >&2
  exit 1
fi
exec ros2 bag play "$bag" --clock "$@"
