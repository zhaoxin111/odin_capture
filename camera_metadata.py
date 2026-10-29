#!/usr/bin/env python3
"""Add optical frame headers and calibrated TF without changing vendor binaries."""
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from camera_frames import (IMAGE_TOPICS, camera_geometry, complete_image_header,
                           static_camera_transform)


class CameraMetadata(Node):
    def __init__(self, calib):
        super().__init__('odin_camera_metadata')
        self.geometry = camera_geometry(calib)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(static_camera_transform(self.geometry))
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.outputs = {}
        self.inputs = []
        for topic in IMAGE_TOPICS:
            cls = CompressedImage if topic.endswith('/compressed') else Image
            pub = self.create_publisher(cls, topic, qos)
            self.outputs[topic] = pub
            self.inputs.append(self.create_subscription(
                cls, topic + '/driver',
                lambda msg, publisher=pub: publisher.publish(complete_image_header(msg)), qos))
        self.get_logger().info('Camera frame and calibrated /tf_static ready: '
                               + self.geometry['camera_frame'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calib', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = CameraMetadata(args.calib)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
