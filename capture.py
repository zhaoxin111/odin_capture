#!/usr/bin/env python3
"""Record an explicit ODIN topic set, validate serialized data, archive with hashes."""
import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

DEPTH_TOPICS = {
    '/odin1/depth_img_competetion': 'sensor_msgs/msg/Image',
    '/odin1/depth_img_competetion_cloud': 'sensor_msgs/msg/PointCloud2',
}

TOPICS = {
    '/odin1/cloud_raw': 'sensor_msgs/msg/PointCloud2',
    '/odin1/imu': 'sensor_msgs/msg/Imu',
    '/odin1/image': 'sensor_msgs/msg/Image',
    '/odin1/image/compressed': 'sensor_msgs/msg/CompressedImage',
    '/odin1/odometry': 'nav_msgs/msg/Odometry',
    '/odin1/odometry_highfreq': 'nav_msgs/msg/Odometry',
    '/odin1/cloud_slam': 'sensor_msgs/msg/PointCloud2',
    **DEPTH_TOPICS,
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
}
REQUIRED = set(TOPICS) - {'/tf_static'}  # This driver publishes extrinsics on /tf.


def required_for_session(session):
    manifest = Path(session) / 'recording_topics.json'
    if manifest.exists():
        return set(json.loads(manifest.read_text())) - {'/tf_static'}
    # Sessions without a manifest recorded JPEG only and no depth completion.
    return REQUIRED - {'/odin1/image'} - set(DEPTH_TOPICS)


def check(session):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    session = Path(session).resolve()
    bag = session if (session / 'metadata.yaml').exists() else session / 'bag'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    counts = collections.Counter()
    first, last, stamps, backwards = {}, {}, {}, collections.Counter()
    empty_payloads = collections.Counter()
    samples = {}
    while reader.has_next():
        topic, data, received = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if hasattr(msg, 'data') and len(msg.data) == 0:
            empty_payloads[topic] += 1
        counts[topic] += 1
        first.setdefault(topic, received)
        last[topic] = received
        header = getattr(msg, 'header', None)
        if header is not None:
            stamp = header.stamp.sec * 10**9 + header.stamp.nanosec
            if stamp < stamps.get(topic, stamp):
                backwards[topic] += 1
            stamps[topic] = stamp
        if topic not in samples:
            samples[topic] = {
                'frame_id': header.frame_id if header else None,
                'header_stamp_ns': stamps.get(topic),
                'payload_bytes': len(data),
            }
            if hasattr(msg, 'fields'):
                samples[topic]['fields'] = [f.name for f in msg.fields]
                samples[topic]['points'] = msg.width * msg.height
            if hasattr(msg, 'format'):
                samples[topic]['format'] = msg.format
            if hasattr(msg, 'encoding'):
                samples[topic].update(width=msg.width, height=msg.height,
                                      encoding=msg.encoding, step=msg.step)
            if hasattr(msg, 'transforms'):
                samples[topic]['transforms'] = [
                    [t.header.frame_id, t.child_frame_id] for t in msg.transforms]
    missing = [t for t in sorted(required_for_session(bag.parent)) if not counts[t]]
    report = {'passed': not missing and not backwards and not empty_payloads,
              'missing': missing, 'empty_payloads': dict(empty_payloads),
              'backwards_header_stamps': dict(backwards), 'topics': {}}
    for t, n in counts.items():
        duration = (last[t] - first[t]) / 1e9
        report['topics'][t] = {'messages': n, 'duration_seconds': duration,
                               'received_hz': (n - 1) / duration if duration else None,
                               'sample': samples[t]}
    (session / 'validation.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    return report['passed']


def probe(seconds=5):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    rclpy.init()
    node = rclpy.create_node('odin_capture_preflight')
    counts = collections.Counter()
    subscriptions = []
    for topic, typename in TOPICS.items():
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=(DurabilityPolicy.TRANSIENT_LOCAL if topic == '/tf_static'
                                     else DurabilityPolicy.VOLATILE))
        subscriptions.append(node.create_subscription(
            get_message(typename), topic,
            lambda msg, t=topic: counts.update([t]), qos))
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    discovery = dict(node.get_topic_names_and_types())
    node.destroy_node()
    rclpy.shutdown()
    print('Live topic counts:', dict(counts), flush=True)
    missing = [t for t in sorted(REQUIRED) if not counts[t]]
    if missing:
        raise RuntimeError('话题无数据，拒绝开始录制: ' + ', '.join(missing))
    return {'probe_seconds': seconds, 'messages': dict(counts), 'discovery': discovery}


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=int, default=60, help='0 表示录制到 Ctrl+C')
    parser.add_argument('--check', type=Path)
    parser.add_argument('--probe', action='store_true')
    parser.add_argument('--no-copy', action='store_true')
    args = parser.parse_args()
    if args.check:
        return 0 if check(args.check) else 1
    if args.seconds < 0:
        parser.error('--seconds must be >= 0')
    live = probe()
    if args.probe:
        return 0
    tools = Path(os.environ['ODIN_TOOLS'])
    ws = Path(os.environ['ODIN_WS'])
    root = Path('/root/odin_bags')
    root.mkdir(exist_ok=True)
    if shutil.disk_usage(root).free < 10 * 1024**3 or shutil.disk_usage('/mnt/d').free < 10 * 1024**3:
        raise RuntimeError('采集前要求 Linux 和 Windows D 盘至少各有 10 GiB 可用空间')
    session = root / time.strftime('%Y%m%d_%H%M%S')
    session.mkdir()
    (session / 'preflight.json').write_text(json.dumps(live, indent=2))
    (session / 'recording_topics.json').write_text(json.dumps(TOPICS, indent=2))
    shutil.copy2(tools / 'capture.yaml', session / 'capture.yaml')
    calib = Path(os.environ['ODIN_CALIB_DIR']) / 'calib.yaml'
    if not calib.exists():
        raise RuntimeError('设备标定文件不存在，拒绝录制')
    shutil.copy2(calib, session / 'calib.yaml')
    repo = ws / 'src/odin_ros_driver'
    (session / 'driver_commit.txt').write_text(subprocess.check_output(
        ['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True))
    (session / 'driver_changes.patch').write_text(subprocess.check_output(
        ['git', '-C', str(repo), 'diff'], text=True))
    qos = {t: {'history': 'keep_last', 'depth': 4000 if t == '/odin1/imu' else 100,
               'reliability': 'reliable',
               'durability': 'transient_local' if t == '/tf_static' else 'volatile'}
           for t in TOPICS}
    import yaml
    (session / 'qos.yaml').write_text(yaml.safe_dump(qos))
    cmd = ['ros2', 'bag', 'record', '-s', 'sqlite3', '-b', str(2 * 1024**3),
           '--max-cache-size', str(256 * 1024**2), '--qos-profile-overrides-path',
           str(session / 'qos.yaml'), '-o', str(session / 'bag'), *TOPICS]
    print('Recording:', session, flush=True)
    with (session / 'recorder.log').open('w') as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        try:
            deadline = time.monotonic() + args.seconds if args.seconds else float('inf')
            while proc.poll() is None and time.monotonic() < deadline:
                time.sleep(0.5)
                if shutil.disk_usage(root).free < 3 * 1024**3 or shutil.disk_usage('/mnt/d').free < 3 * 1024**3:
                    print('剩余空间不足 3 GiB，停止录制', flush=True)
                    break
        except KeyboardInterrupt:
            print('正在关闭 bag，请等待校验完成。', flush=True)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                raise RuntimeError('录制器尚未完成关闭，保留原始目录，暂不复制')
    if proc.returncode not in (0, 130, -signal.SIGINT):
        raise RuntimeError(f'录制器退出码 {proc.returncode}，请检查 {session}/recorder.log')
    if not check(session):
        raise RuntimeError(f'bag 校验未通过，数据保留在 {session}')
    logs = sorted((ws / 'logs').glob('driver_*.log'))
    if logs:
        shutil.copy2(logs[-1], session / 'driver.log')
    manifest = {str(p.relative_to(session)): sha256(p)
                for p in sorted(session.rglob('*')) if p.is_file()}
    (session / 'sha256.json').write_text(json.dumps(manifest, indent=2))
    if not args.no_copy:
        destination = Path('/mnt/d/workspace/odin_bags') / session.name
        size = sum(p.stat().st_size for p in session.rglob('*') if p.is_file())
        if shutil.disk_usage(destination.parent.parent).free < size + 3 * 1024**3:
            raise RuntimeError(f'Windows 空间不足，原始数据已保存在 {session}')
        shutil.copytree(session, destination)
        for name, digest in manifest.items():
            if sha256(destination / name) != digest:
                raise RuntimeError('副本校验失败: ' + name)
        print('Windows copy verified:', destination, flush=True)
    print('采集完成:', session, flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
