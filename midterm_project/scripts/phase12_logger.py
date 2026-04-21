#!/usr/bin/env python3
"""
Phase 12 logger — records rover ground truth, raw odom, and TRN-corrected
pose to /tmp/phase12_poses.csv for post-run localization error analysis.

Three sources, all written with wall-clock timestamps:
  gt   — gz model -m rover -p (world frame, converted to rig-relative)
  odom — /rover/odom           (DiffDrive, start-relative)
  trn  — /rover/trn_pose       (TRN-corrected, start-relative)

Run alongside the demo; Ctrl-C or SIGTERM stops it cleanly.
"""

import csv
import os
import subprocess
import re
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

OUTPUT = os.environ.get('P12_LOG', '/tmp/phase12_poses.csv')
RIG_X = 5.0
RIG_Y = -20.0


class Phase12Logger(Node):
    def __init__(self, writer, lock):
        super().__init__('phase12_logger')
        self._w = writer
        self._lock = lock

        gz_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Odometry, '/rover/odom', self._odom_cb, gz_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/rover/trn_pose',
            self._trn_cb,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
        )

        # 2 Hz GT poll (matches groundtruth_logger rate)
        self.create_timer(0.5, self._gt_cb)
        self.get_logger().info(f'Phase12Logger started — writing to {OUTPUT}')

    def _write(self, source, x, y):
        with self._lock:
            self._w.writerow([f'{time.time():.4f}', source,
                              f'{x:.6f}', f'{y:.6f}'])

    def _odom_cb(self, msg: Odometry):
        self._write('odom',
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y)

    def _trn_cb(self, msg: PoseWithCovarianceStamped):
        self._write('trn',
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y)

    def _gt_cb(self):
        try:
            r = subprocess.run(
                ['gz', 'model', '-m', 'rover', '-p'],
                capture_output=True, text=True, timeout=1.5)
            lines = [l.strip() for l in r.stdout.splitlines()
                     if l.strip().startswith('[')]
            if len(lines) >= 1:
                nums = re.findall(
                    r'(-?[0-9]+\.?[0-9]*(?:e[+-]?[0-9]+)?)', lines[0])
                if len(nums) >= 2:
                    wx, wy = float(nums[0]), float(nums[1])
                    self._write('gt', wx - RIG_X, wy - RIG_Y)
        except Exception:
            pass


def main():
    rclpy.init()

    with open(OUTPUT, 'w', newline='') as f:
        lock = threading.Lock()
        w = csv.writer(f)
        w.writerow(['wall_time', 'source', 'x', 'y'])
        f.flush()

        node = Phase12Logger(w, lock)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    print(f'[phase12_logger] done — {OUTPUT}', flush=True)


if __name__ == '__main__':
    main()
