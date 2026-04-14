#!/usr/bin/env python3
"""
Demo Image Saver — saves downward camera frames and drone poses.
Subscribes to:
  /drone/down_mono  (sensor_msgs/Image)
  /drone/odom       (nav_msgs/Odometry)
Saves every 3rd frame as PNG in ~/midterm_flight_data/rgb/.
Writes poses.json on shutdown.
"""

import json
import os
import signal

import cv2
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class DemoImageSaver(Node):

    def __init__(self):
        super().__init__('demo_image_saver')

        self.output_dir = os.path.expanduser('~/midterm_flight_data/rgb')
        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.saved_count = 0
        self.save_every_nth = 3
        self.poses = []
        self.current_pose = None   # updated from /drone/odom

        self.img_sub = self.create_subscription(
            Image, '/drone/down_mono', self._image_cb, _QOS)
        self.odom_sub = self.create_subscription(
            Odometry, '/drone/odom', self._odom_cb, _QOS)

        self.get_logger().info(
            'DemoImageSaver ready — /drone/down_mono + /drone/odom')

    # ────────────────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current_pose = {
            'x': round(p.x, 4), 'y': round(p.y, 4), 'z': round(p.z, 4),
            'qx': round(q.x, 6), 'qy': round(q.y, 6),
            'qz': round(q.z, 6), 'qw': round(q.w, 6),
        }

    # ────────────────────────────────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.save_every_nth != 0:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        fname = os.path.join(self.output_dir, f'frame_{self.saved_count:04d}.png')
        cv2.imwrite(fname, cv_img)

        if self.current_pose is not None:
            self.poses.append({'frame': self.saved_count, **self.current_pose})

        self.saved_count += 1
        if self.saved_count % 10 == 0:
            self.get_logger().info(f'Saved {self.saved_count} frames')

    # ────────────────────────────────────────────────────────────────────────
    def shutdown(self):
        poses_file = os.path.expanduser('~/midterm_flight_data/poses.json')
        with open(poses_file, 'w') as f:
            json.dump(self.poses, f, indent=2)
        self.get_logger().info(
            f'Saved {self.saved_count} frames, {len(self.poses)} poses → {poses_file}')


def main(args=None):
    rclpy.init(args=args)
    node = DemoImageSaver()
    # Handle SIGTERM gracefully so poses.json gets written
    signal.signal(signal.SIGTERM, lambda s, f: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
