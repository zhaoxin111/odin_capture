#!/usr/bin/env bash
# Source this file from any directory.
export ODIN_WS="${ODIN_WS:-/root/odin_ws}"
export ODIN_TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ODIN_CALIB_DIR="${ODIN_CALIB_DIR:-$ODIN_WS/runtime}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-71}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ODIN_PYTHON="${ODIN_PYTHON:-/usr/bin/python3}"
# Playback only needs standard ROS messages, not the hardware driver overlay.
# Prefer an explicit/active distro, then the original Humble environment.
odin_ros_distro="${ODIN_ROS_DISTRO:-${ROS_DISTRO:-}}"
if [[ -z "$odin_ros_distro" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    odin_ros_distro=humble
  elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
    odin_ros_distro=jazzy
  fi
fi
if [[ ! -f "/opt/ros/$odin_ros_distro/setup.bash" ]]; then
  echo '未找到 ROS 环境，请设置 ODIN_ROS_DISTRO（humble 或 jazzy）。' >&2
  return 1
fi
source "/opt/ros/$odin_ros_distro/setup.bash"
if [[ -f "$ODIN_WS/install/setup.bash" ]]; then
  source "$ODIN_WS/install/setup.bash"
fi
unset odin_ros_distro
