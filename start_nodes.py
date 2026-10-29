#!/usr/bin/env python3
"""Launch the driver and configured processing nodes with shared config/calibration."""
import os
import sys
import time
from pathlib import Path

import yaml
from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from calibration import CalibrationGate
from camera_frames import IMAGE_TOPICS


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
                  remappings=[(topic, topic + '/driver') for topic in IMAGE_TOPICS],
                  **SHUTDOWN_TIMEOUTS)]
    nodes.append(ExecuteProcess(
        cmd=[sys.executable, str(Path(__file__).with_name('camera_metadata.py')),
             '--calib', str(calib)], output='screen', **SHUTDOWN_TIMEOUTS))
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
    # A previous run may have left an empty/truncated file. Preserve it for
    # diagnosis and force the SDK to retrieve this device's calibration again,
    # including when the device resumes from STREAM_STOPPED.
    if calib.exists():
        backup = calib.with_name(f'calib.previous.{time.time_ns()}.yaml')
        calib.rename(backup)
        print(f'Previous calibration preserved: {backup}', flush=True)
    gate = CalibrationGate(calib)
    deadline = time.monotonic() + 60

    def wait_for_calibration(context):
        if context.is_shutdown:
            return []
        if gate.ready():
            print('设备标定校验通过，启动处理节点。', flush=True)
            return nodes[1:]
        if time.monotonic() >= deadline:
            reason = f'等待设备标定超时（60 秒）: {gate.error}; {calib}'
            print(reason, flush=True)
            return [EmitEvent(event=Shutdown(reason=reason))]
        return [TimerAction(period=0.5, actions=[OpaqueFunction(function=wait_for_calibration)])]

    print(f'Config: {config}\nCalibration: {calib}\n等待设备获取完整标定（最多 60 秒）…', flush=True)
    return LaunchDescription(handlers + [nodes[0], TimerAction(
        period=0.5, actions=[OpaqueFunction(function=wait_for_calibration)])])


if __name__ == '__main__':
    service = LaunchService()
    service.include_launch_description(make_description())
    raise SystemExit(service.run())
