#!/usr/bin/env bash
set -eo pipefail
export ODIN_WS="${ODIN_WS:-/root/odin_ws}"
source /opt/ros/humble/setup.bash
cd "$ODIN_WS"
cp src/odin_ros_driver/package_ros2.xml src/odin_ros_driver/package.xml
# 8 GiB WSL: avoid exhausting memory when compiling PCL/OpenCV code.
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon build --packages-select odin_ros_driver --executor sequential \
  --cmake-args -DBUILD_SYSTEM=ROS2 -DCMAKE_BUILD_TYPE=Release
