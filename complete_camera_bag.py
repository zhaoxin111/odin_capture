#!/usr/bin/env python3
"""Create a separate bag copy with optical image headers and camera static TF."""
import argparse
import collections
import json
from pathlib import Path
import shutil
import tempfile

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

from camera_frames import (CAMERA_FRAME, IMAGE_TOPICS, camera_geometry,
                           complete_image_header, static_camera_transform)
from capture import check, write_hashes


STATIC_QOS = '''- history: 1
  depth: 1
  reliability: 1
  durability: 1
  deadline:
    sec: 9223372036
    nsec: 854775807
  lifespan:
    sec: 9223372036
    nsec: 854775807
  liveliness: 1
  liveliness_lease_duration:
    sec: 9223372036
    nsec: 854775807
  avoid_ros_namespace_conventions: false
'''


def complete_session(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination == source or source in destination.parents:
        raise ValueError('输出必须是独立目录')
    if destination.exists():
        raise FileExistsError(destination)
    geometry = camera_geometry(source / 'calib.yaml')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source / 'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    if not reader.has_next():
        raise ValueError('bag 没有消息')
    first = reader.read_next()
    metadata = reader.get_all_topics_and_types()
    types = {t.name: get_message(t.type) for t in metadata}
    # The input remains read-only. Reports and hashes are regenerated for the copy.
    shutil.copytree(source, destination, ignore=shutil.ignore_patterns(
        'bag', 'sha256.json', 'validation.json', 'replay_validation.json',
        'camera_completion.json', 'camera_frames.json'))
    staging = tempfile.TemporaryDirectory(prefix='odin_camera_bag_')
    staged_bag = Path(staging.name) / 'bag'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(staged_bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    for topic in metadata:
        writer.create_topic(topic)
    if '/tf_static' not in types:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name='/tf_static', type='tf2_msgs/msg/TFMessage', serialization_format='cdr',
            offered_qos_profiles=STATIC_QOS))
    writer.write('/tf_static', serialize_message(TFMessage(
        transforms=[static_camera_transform(geometry)])), first[2])
    changed = collections.Counter()

    def write_record(record):
        topic, data, timestamp = record
        if topic in IMAGE_TOPICS:
            msg = deserialize_message(data, types[topic])
            data = serialize_message(complete_image_header(msg))
            changed[topic] += 1
        elif topic == '/tf_static':
            msg = deserialize_message(data, types[topic])
            if any(t.child_frame_id == CAMERA_FRAME for t in msg.transforms):
                raise ValueError('输入已含相机静态 TF，请直接使用原 bag 或检查变换冲突')
        writer.write(topic, data, timestamp)

    write_record(first)
    while reader.has_next():
        write_record(reader.read_next())
    del writer
    del reader
    shutil.copytree(staged_bag, destination / 'bag')
    staging.cleanup()
    manifest = source / 'recording_topics.json'
    topics = json.loads(manifest.read_text()) if manifest.exists() else {
        t.name: t.type for t in metadata}
    topics['/tf_static'] = 'tf2_msgs/msg/TFMessage'
    (destination / 'recording_topics.json').write_text(json.dumps(topics, indent=2))
    (destination / 'camera_frames.json').write_text(json.dumps(geometry, indent=2))
    (destination / 'camera_completion.json').write_text(json.dumps({
        'source': str(source), 'images_completed': dict(changed),
        'added_static_transform': [geometry['lidar_frame'], CAMERA_FRAME],
        'preserved': 'Image payloads, sensor stamps, original receive times, non-image messages',
    }, indent=2))
    if not check(destination):
        raise RuntimeError(f'补全副本校验失败，保留诊断目录: {destination}')
    write_hashes(destination)
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    print('补全副本:', complete_session(args.source, args.destination))
