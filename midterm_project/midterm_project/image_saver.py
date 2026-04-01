#!/usr/bin/env python3
"""
Phase 61-65: Image Saver Node
- Subscribes to /drone/front_rgb and /drone/front_depth
- Saves every Nth RGB frame as PNG
- Saves corresponding depth frames as 16-bit PNG
- Records odometry (position + quaternion) per saved frame
- Exports poses.json on shutdown
- Verifies inter-frame pixel difference > 1.0 (no identical frames)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleOdometry

import cv2
import numpy as np
import os
import json
from cv_bridge import CvBridge


class ImageSaver(Node):

    def __init__(self):
        super().__init__('image_saver')

        # Parameters
        self.declare_parameter('save_every_nth', 10)
        self.declare_parameter('output_dir',
            os.path.expanduser(
                '~/ses598-space-robotics-and-ai-2026/midterm_project/data/images'))
        self.declare_parameter('min_frame_diff', 1.0)

        self.save_every_nth = self.get_parameter('save_every_nth').value
        self.output_dir     = self.get_parameter('output_dir').value
        self.min_frame_diff = self.get_parameter('min_frame_diff').value

        # Create output directories
        self.rgb_dir   = os.path.join(self.output_dir, 'rgb')
        self.depth_dir = os.path.join(self.output_dir, 'depth')
        os.makedirs(self.rgb_dir,   exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)

        # QoS
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/drone/front_rgb',
            self.rgb_callback, qos_sensor)
        self.depth_sub = self.create_subscription(
            Image, '/drone/front_depth',
            self.depth_callback, qos_sensor)
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_callback, qos_px4)

        # State
        self.bridge         = CvBridge()
        self.frame_count    = 0
        self.saved_count    = 0
        self.last_rgb       = None
        self.last_depth     = None
        self.last_odom      = None
        self.poses          = []

        self.get_logger().info(
            f'ImageSaver initialized — saving every {self.save_every_nth}th frame '
            f'to {self.output_dir}')

    def odom_callback(self, msg: VehicleOdometry):
        self.last_odom = {
            'position': [float(msg.position[0]),
                         float(msg.position[1]),
                         float(msg.position[2])],
            'quaternion': [float(msg.q[0]),
                           float(msg.q[1]),
                           float(msg.q[2]),
                           float(msg.q[3])]
        }

    def depth_callback(self, msg: Image):
        try:
            self.last_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def rgb_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'RGB conversion failed: {e}')
            return

        self.frame_count += 1

        # Only save every Nth frame
        if self.frame_count % self.save_every_nth != 0:
            return

        # Check inter-frame pixel difference — skip if identical
        if self.last_rgb is not None:
            diff = np.mean(np.abs(
                frame.astype(np.float32) - self.last_rgb.astype(np.float32)))
            if diff < self.min_frame_diff:
                self.get_logger().debug(
                    f'Frame skipped — diff={diff:.3f} < {self.min_frame_diff}')
                return

        # Save RGB
        rgb_path = os.path.join(self.rgb_dir, f'frame_{self.saved_count:05d}.png')
        cv2.imwrite(rgb_path, frame)

        # Save depth if available
        if self.last_depth is not None:
            depth_path = os.path.join(
                self.depth_dir, f'frame_{self.saved_count:05d}.png')
            cv2.imwrite(depth_path, self.last_depth)

        # Record pose
        pose_entry = {
            'frame_id': self.saved_count,
            'ros_frame': self.frame_count,
            'rgb_path':  rgb_path,
            'depth_path': os.path.join(
                self.depth_dir, f'frame_{self.saved_count:05d}.png'),
            'odom': self.last_odom
        }
        self.poses.append(pose_entry)

        self.last_rgb = frame.copy()
        self.saved_count += 1

        if self.saved_count % 10 == 0:
            self.get_logger().info(
                f'Saved {self.saved_count} frames '
                f'(total received: {self.frame_count})')

    def save_poses(self):
        poses_path = os.path.join(self.output_dir, 'poses.json')
        with open(poses_path, 'w') as f:
            json.dump(self.poses, f, indent=2)
        self.get_logger().info(
            f'Saved {len(self.poses)} poses to {poses_path}')

    def destroy_node(self):
        self.save_poses()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_poses()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
