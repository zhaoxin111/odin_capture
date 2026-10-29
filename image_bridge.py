#!/usr/bin/env python3
"""Decode older JPEG-only bags for RViz and standard Image subscribers."""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class ImageBridge(Node):
    def __init__(self):
        super().__init__('odin_jpeg_decoder')
        self.bridge = CvBridge()
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Image, '/odin1/image', qos)
        self.subscription = self.create_subscription(
            CompressedImage, '/odin1/image/compressed', self.decode, qos)
        self.logged = False
        self.get_logger().info('JPEG decoder ready; preserves the original image header.')

    def decode(self, msg):
        # New bags and the live driver already publish Image. Avoid duplicate frames.
        if self.count_publishers('/odin1/image') > 1:
            return
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            out = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            out.header = msg.header
            self.publisher.publish(out)
            if not self.logged:
                self.get_logger().info(f'Decoded {out.width}x{out.height} bgr8 on /odin1/image')
                self.logged = True
        except Exception as exc:
            self.get_logger().error(f'Could not decode image: {exc}')


def main():
    rclpy.init()
    node = ImageBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
