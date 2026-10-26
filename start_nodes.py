#!/usr/bin/env python3
"""Launch the driver and configured processing nodes with shared config/calibration."""
import os
from pathlib import Path

import yaml
from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


# Allow graceful shutdown, then bound a stuck vendor USB cleanup call.
SHUTDOWN_TIMEOUTS = {'sigterm_timeout': '8', 'sigkill_timeout': '2'}


def make_description():
    config = Path(os.environ['ODIN_TOOLS']) / 'capture.yaml'
    calib = Path(os.environ['ODIN_CALIB_DIR']) / 'calib.yaml'
    keys = yaml.safe_load(config.read_text())['register_keys']
    enabled = lambda key: keys.get(key, 0) == 1
    requirements = []
    if enabled('senddepth'):
        requirements += ['senddtof', 'sendrgb']
    if enabled('sendreprojection'):
        requirements += ['sendcloudslam', 'sendodom']
    if enabled('sendoverlay'):
        requirements += ['sendreprojection', 'sendrgb', 'sendrgbundistort']
    missing = sorted({key for key in requirements if not enabled(key)})
    if missing:
        raise ValueError('capture.yaml: 请先开启依赖: ' + ', '.join(missing))

    shared = {'config_file': str(config), 'calib_file_path': str(calib)}
    nodes = [Node(package='odin_ros_driver', executable='host_sdk_sample',
                  output='screen', parameters=[{'config_file': str(config)}],
                  **SHUTDOWN_TIMEOUTS)]
    for flag, executable in [('senddepth', 'pcd2depth_ros2_node'),
                             ('sendreprojection', 'cloud_reprojection_ros2_node')]:
        if enabled(flag):
            nodes.append(Node(package='odin_ros_driver', executable=executable,
                              output='screen', parameters=[shared],
                              **SHUTDOWN_TIMEOUTS))
    if enabled('sendoverlay'):
        params = {'register_keys.' + key: value for key, value in keys.items()
                  if key.startswith('overlay_')}
        nodes.append(Node(package='odin_ros_driver', executable='image_overlay_node',
                          output='screen', parameters=[params],
                          **SHUTDOWN_TIMEOUTS))

    # Register before starting: an early failure must stop the entire group.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='ODIN node exited'))]))
        for node in nodes]
    print(f'Config: {config}\nCalibration: {calib}', flush=True)
    return LaunchDescription(handlers + nodes)


if __name__ == '__main__':
    service = LaunchService()
    service.include_launch_description(make_description())
    raise SystemExit(service.run())
