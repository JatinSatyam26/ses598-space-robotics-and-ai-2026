#!/usr/bin/env python3
"""
Demo Image Saver — no odometry dependency.
Subscribes to /drone/down_mono, saves every 3rd frame as PNG.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

import cv2
import numpy as np
import os
import signal
from cv_bridge import CvBridge


class DemoImageSaver(Node):

    def __init__(self):
        super().__init__('demo_image_saver')

        self.output_dir = os.path.expanduser('~/midterm_flight_data/rgb')
        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.saved_count = 0
        self.save_every_nth = 3

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            Image,
            '/drone/down_mono',
            self.image_callback,
            qos
        )

        self.get_logger().info('DemoImageSaver ready — subscribing to /drone/down_mono')

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.save_every_nth != 0:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Bridge error: {e}')
            return

        fname = os.path.join(self.output_dir, f'frame_{self.saved_count:04d}.png')
        cv2.imwrite(fname, cv_img)
        self.saved_count += 1

        if self.saved_count % 10 == 0:
            self.get_logger().info(f'Saved {self.saved_count} frames')

    def shutdown(self):
        self.get_logger().info(f'Total frames saved: {self.saved_count}')


def main(args=None):
    rclpy.init(args=args)
    node = DemoImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
