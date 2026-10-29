"""Regression tests for host clock steps, using small real sqlite ROS bags."""
from pathlib import Path
import sqlite3
import shutil
import tempfile
import unittest

import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image

from bag_timing import bag_files, read_database, receive_time_plan, repair_session


class TimingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.source = Path(self.temp.name) / 'original'
        self.source.mkdir()

    def make_bag(self, sensor_stamps=(1, 2, 3, 4)):
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(self.source / 'bag'), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        writer.create_topic(rosbag2_py.TopicMetadata(name='/odin1/image',
                            type='sensor_msgs/msg/Image', serialization_format='cdr'))
        for stamp, received in zip(sensor_stamps, (1000000000, 2000000000, 500000000, 1500000000)):
            message = Image()
            message.header.stamp.sec = stamp
            message.data = [1, 2, 3]
            writer.write('/odin1/image', serialize_message(message), received)
        del writer

    def rows(self, session):
        result = []
        for file in bag_files(session / 'bag'):
            with read_database(file) as db:
                result.extend(db.execute('SELECT timestamp,data FROM messages ORDER BY id'))
        return result

    def test_repair_preserves_original_and_payloads(self):
        import json
        from capture import check
        self.make_bag()
        (self.source / 'recording_topics.json').write_text(json.dumps({
            '/odin1/image': 'sensor_msgs/msg/Image'}))
        self.assertFalse(check(self.source))
        original = self.rows(self.source)
        plan = receive_time_plan(self.source / 'bag')
        self.assertEqual(len(plan['receive_time_rollbacks']), 1)
        destination = Path(self.temp.name) / 'repaired'
        repair_session(self.source, destination)
        repaired = self.rows(destination)
        self.assertEqual(self.rows(self.source), original)
        self.assertEqual([row[1] for row in original], [row[1] for row in repaired])
        self.assertEqual([row[0] for row in repaired],
                         [1000000000, 2000000000, 2000000001, 3000000001])
        self.assertFalse(receive_time_plan(destination / 'bag')['receive_time_rollbacks'])
        self.assertTrue(check(destination))
        import yaml
        metadata = yaml.safe_load((destination / 'bag/metadata.yaml').read_text())
        self.assertEqual(metadata['rosbag2_bagfile_information']['duration']['nanoseconds'], 2000000001)

    def test_sensor_rollback_is_rejected(self):
        self.make_bag(sensor_stamps=(1, 3, 2, 4))
        destination = Path(self.temp.name) / 'rejected'
        with self.assertRaisesRegex(ValueError, '传感器时间戳'):
            repair_session(self.source, destination)
        self.assertFalse(destination.exists())

    def test_rollback_at_split_boundary(self):
        import yaml
        self.make_bag()
        first = self.source / 'bag/bag_0.db3'
        second = self.source / 'bag/bag_1.db3'
        shutil.copy2(first, second)
        with sqlite3.connect(first) as db:
            db.execute('DELETE FROM messages WHERE id>2')
        with sqlite3.connect(second) as db:
            db.execute('DELETE FROM messages WHERE id<=2')
        metadata_path = self.source / 'bag/metadata.yaml'
        metadata = yaml.safe_load(metadata_path.read_text())
        metadata['rosbag2_bagfile_information']['relative_file_paths'] = [first.name, second.name]
        metadata_path.write_text(yaml.safe_dump(metadata))
        destination = Path(self.temp.name) / 'split_repaired'
        repair_session(self.source, destination)
        self.assertEqual([row[0] for row in self.rows(destination)],
                         [1000000000, 2000000000, 2000000001, 3000000001])
        self.assertFalse(receive_time_plan(destination / 'bag')['receive_time_rollbacks'])


if __name__ == '__main__':
    unittest.main()
