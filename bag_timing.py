"""Diagnose host clock rollback and repair receive times in a separate copy."""
import json
from pathlib import Path
import shutil
import sqlite3
import struct
import subprocess
import tempfile

import yaml


HEADER_TYPES = {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
                'sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/Imu',
                'nav_msgs/msg/Odometry'}


def bag_files(bag):
    metadata = yaml.safe_load((Path(bag) / 'metadata.yaml').read_text())
    return [Path(bag) / name for name in
            metadata['rosbag2_bagfile_information']['relative_file_paths']]


def read_database(path):
    return sqlite3.connect(Path(path).resolve().as_uri() + '?mode=ro', uri=True)


def receive_time_plan(bag):
    """Use insertion order, which survives a backwards host wall clock step."""
    previous = None
    offset = 0
    jumps, segments = [], []
    for path in bag_files(bag):
        with read_database(path) as db:
            start_id = None
            for row_id, timestamp in db.execute('SELECT id,timestamp FROM messages ORDER BY id'):
                if start_id is None:
                    start_id = row_id
                corrected = timestamp + offset
                if previous is not None and corrected < previous:
                    segments.append((path.name, start_id, row_id, offset))
                    adjustment = previous - corrected + 1
                    jumps.append({'file': path.name, 'message_id': row_id,
                                  'backwards_ns': adjustment - 1,
                                  'original_receive_ns': timestamp})
                    offset += adjustment
                    corrected += adjustment
                    start_id = row_id
                previous = corrected
            if start_id is not None:
                segments.append((path.name, start_id, None, offset))
    return {'receive_time_rollbacks': jumps, 'total_adjustment_ns': offset,
            'segments': segments}


def storage_header_backwards(bag):
    """Check supported CDR Header stamps without loading image/cloud payloads."""
    previous, backwards = {}, {}
    for path in bag_files(bag):
        with read_database(path) as db:
            topics = {row[0]: row[1] for row in db.execute(
                'SELECT id,name,type,serialization_format FROM topics')
                if row[2] in HEADER_TYPES and row[3] == 'cdr'}
            for topic_id, data in db.execute(
                    'SELECT topic_id,substr(data,1,12) FROM messages ORDER BY id'):
                if topic_id not in topics:
                    continue
                if len(data) < 12 or data[:2] not in (b'\x00\x01', b'\x00\x00'):
                    raise ValueError('不支持的 CDR 消息头，不能自动修正接收时间')
                sec, ns = struct.unpack_from('<iI' if data[1] == 1 else '>iI', data, 4)
                stamp = sec * 10**9 + ns
                topic = topics[topic_id]
                if stamp < previous.get(topic, stamp):
                    backwards[topic] = backwards.get(topic, 0) + 1
                previous[topic] = stamp
    return backwards


def repair_session(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination == source or source in destination.parents:
        raise ValueError('修正副本必须位于原始采集目录之外')
    if destination.exists():
        raise FileExistsError(f'修正目标已存在: {destination}')
    plan = receive_time_plan(source / 'bag')
    if not plan['receive_time_rollbacks']:
        raise ValueError('未发现主机接收时间倒退，无需修正')
    print('检查按写入顺序排列的传感器时间戳…', flush=True)
    backwards = storage_header_backwards(source / 'bag')
    if backwards:
        raise ValueError(f'传感器时间戳也有倒退，不能自动修正: {backwards}')
    size = sum(p.stat().st_size for p in source.rglob('*') if p.is_file())
    destination.parent.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(destination.parent).free < size + 3 * 1024**3:
        raise RuntimeError('空间不足以保留原始数据并创建修正副本')
    largest = max(path.stat().st_size for path in bag_files(source / 'bag'))
    if shutil.disk_usage(tempfile.gettempdir()).free < largest + 3 * 1024**3:
        raise RuntimeError('Linux 临时目录空间不足以处理一个分包')
    print(f'保留原始数据，创建接收时间修正副本: {destination}', flush=True)

    # Update on the Linux filesystem: rewriting large SQLite rows directly on
    # a Windows mounted drive is much slower than copying the finished file.
    with tempfile.TemporaryDirectory(prefix='odin-bag-repair-') as staging:
        def copy_file(original, copied):
            original = Path(original)
            if original.parent != source / 'bag' or original.suffix != '.db3':
                return shutil.copy2(original, copied)
            print(f'修正并复制分包: {original.name}', flush=True)
            local = Path(staging) / original.name
            shutil.copy2(original, local)
            with sqlite3.connect(local) as db:
                for filename, start, end, offset in plan['segments']:
                    if filename != original.name or not offset:
                        continue
                    db.execute('UPDATE messages SET timestamp=timestamp+? WHERE id>=? AND id<?',
                               (offset, start, end if end is not None else 2**63 - 1))
            result = shutil.copy2(local, copied)
            local.unlink()
            return result

        shutil.copytree(source, destination, copy_function=copy_file)
    # Existing hashes/validation describe the unmodified data, so keep the
    # original validation separately and regenerate the final checks and hashes.
    validation = destination / 'validation.json'
    if validation.exists():
        validation.rename(destination / 'validation.original.json')
    (destination / 'sha256.json').unlink(missing_ok=True)
    # Rebuild both the per-file bounds and the bag-wide metadata.
    subprocess.run(['ros2', 'bag', 'reindex', '-s', 'sqlite3',
                    str(destination / 'bag')], check=True)
    plan.update(original_session=str(source), storage_order_header_backwards=backwards,
                method='Cumulative offset after each receive-time rollback; message payloads unchanged. '
                       'The backwards step is replaced by 1 ns; exact elapsed time at the step is unknown.')
    (destination / 'timestamp_repair.json').write_text(json.dumps(plan, indent=2))
    return destination
