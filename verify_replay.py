#!/usr/bin/env python3
"""Verify an actual ROS replay with the live hardware driver stopped."""
import collections
import json
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from rosbag2_interfaces.srv import Resume
from capture import TOPICS, required_for_session


def main():
    session = Path(sys.argv[1]).resolve()
    bag = session / 'bag' if (session / 'bag').exists() else session
    required = required_for_session(bag.parent)
    rclpy.init()
    node = rclpy.create_node('odin_replay_verification')
    counts = collections.Counter()
    subscriptions = []
    for topic, typename in TOPICS.items():
        qos = QoSProfile(depth=4000, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if topic == '/tf_static'
                         else DurabilityPolicy.VOLATILE)
        subscriptions.append(node.create_subscription(
            get_message(typename), topic,
            lambda msg, t=topic: counts.update([t]), qos))
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if any(name == 'lydros_node' for name in node.get_node_names()):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Stop the live driver before testing replay.')
    with (session / 'replay.log').open('w') as log:
        player = subprocess.Popen(['ros2', 'bag', 'play', str(bag), '--clock',
                                   '--rate', '4', '--start-paused',
                                   '--disable-keyboard-controls',
                                   '--wait-for-all-acked', '5000'],
                                  stdout=log, stderr=subprocess.STDOUT)
        resume = node.create_client(Resume, '/rosbag2_player/resume')
        ready_deadline = time.monotonic() + 15
        while time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if resume.service_is_ready() and all(node.count_publishers(t) for t in required):
                break
        else:
            player.terminate()
            player.wait(timeout=10)
            raise RuntimeError('Playback publishers did not become ready.')
        # Let reliable DDS endpoints match before the first high-frequency message.
        ready_deadline = time.monotonic() + 3
        while time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        future = resume.call_async(Resume.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.exception():
            player.terminate()
            player.wait(timeout=10)
            raise RuntimeError('Could not resume playback.')
        deadline = time.monotonic() + 120
        while player.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if player.poll() is None:
            player.terminate()
            player.wait(timeout=10)
            raise RuntimeError('Replay timeout; inspect replay.log')
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    expected = json.loads((session / 'validation.json').read_text())['topics']
    differences = {t: {'expected': expected[t]['messages'], 'received': counts[t]}
                   for t in required if counts[t] != expected[t]['messages']}
    report = {'passed': player.returncode == 0 and not differences,
              'player_exit_code': player.returncode, 'received_messages': dict(counts),
              'count_differences': differences}
    (session / 'replay_validation.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
