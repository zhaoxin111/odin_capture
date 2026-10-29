#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
exec "$ODIN_PYTHON" "$ODIN_TOOLS/image_bridge.py"
