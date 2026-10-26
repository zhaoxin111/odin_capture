#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
mkdir -p "$ODIN_CALIB_DIR" "$ODIN_WS/logs"
if ! lsusb -d 2207:0019 | grep -q .; then
  echo '未发现 ODIN1。请先在 Windows 执行 attach.ps1，再重试。' >&2
  exit 1
fi
# Keep the log pipe alive while the driver handles Ctrl+C and writes shutdown logs.
exec > >(trap '' INT TERM; tee -a "$ODIN_WS/logs/driver_$(date +%Y%m%d_%H%M%S).log") 2>&1
exec python3 "$ODIN_TOOLS/start_nodes.py"