#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
exec python3 "$ODIN_TOOLS/image_bridge.py"
