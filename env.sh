#!/usr/bin/env bash
# Source this file from any directory.
export ODIN_WS="${ODIN_WS:-/root/odin_ws}"
export ODIN_TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ODIN_CALIB_DIR="${ODIN_CALIB_DIR:-$ODIN_WS/runtime}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-71}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source "$ODIN_WS/install/setup.bash"
