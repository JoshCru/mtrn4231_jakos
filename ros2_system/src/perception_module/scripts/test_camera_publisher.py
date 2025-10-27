#!/usr/bin/env python3
"""
Simple test publisher to simulate RealSense camera topics for testing rgbd_camera_node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np


class TestCameraPublisher(Node):
    def __init__(self):
        super().__init__('test_camera_publisher')

        # Publishers
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)

        # Timer to publish at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.frame_count = 0

        self.get_logger().info('Test camera publisher started')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  /camera/color/image_raw')
        self.get_logger().info('  /camera/depth/image_raw')
        self.get_logger().info('  /camera/color/camera_info')
        self.get_logger().info('  /camera/depth/camera_info')

    def timer_callback(self):
        timestamp = self.get_clock().now().to_msg()

        # Create color image (640x480 RGB8)
        color_msg = Image()
        color_msg.header.stamp = timestamp
        color_msg.header.frame_id = 'camera_color_optical_frame'
        color_msg.height = 480
        color_msg.width = 640
        color_msg.encoding = 'rgb8'
        color_msg.is_bigendian = False
        color_msg.step = 640 * 3

        # Create a gradient pattern
        color_data = np.zeros((480, 640, 3), dtype=np.uint8)
        color_data[:, :, 0] = (self.frame_count * 5) % 255  # R
        color_data[:, :, 1] = np.linspace(0, 255, 640, dtype=np.uint8)  # G
        color_data[:, :, 2] = np.linspace(0, 255, 480, dtype=np.uint8).reshape(-1, 1)  # B
        color_msg.data = color_data.tobytes()

        # Create depth image (640x480 16UC1)
        depth_msg = Image()
        depth_msg.header.stamp = timestamp
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        depth_msg.height = 480
        depth_msg.width = 640
        depth_msg.encoding = '16UC1'
        depth_msg.is_bigendian = False
        depth_msg.step = 640 * 2

        # Create depth data (values in mm)
        depth_data = np.random.randint(500, 2000, (480, 640), dtype=np.uint16)
        depth_msg.data = depth_data.tobytes()

        # Create camera info messages
        color_info = self.create_camera_info(timestamp, 'camera_color_optical_frame')
        depth_info = self.create_camera_info(timestamp, 'camera_depth_optical_frame')

        # Publish all messages
        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        self.color_info_pub.publish(color_info)
        self.depth_info_pub.publish(depth_info)

        self.frame_count += 1

        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Published frame {self.frame_count}')

    def create_camera_info(self, timestamp, frame_id):
        info = CameraInfo()
        info.header.stamp = timestamp
        info.header.frame_id = frame_id
        info.height = 480
        info.width = 640

        # Typical RealSense D435 intrinsics
        info.k = [615.0, 0.0, 320.0,
                  0.0, 615.0, 240.0,
                  0.0, 0.0, 1.0]

        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        return info


def main(args=None):
    rclpy.init(args=args)
    node = TestCameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
