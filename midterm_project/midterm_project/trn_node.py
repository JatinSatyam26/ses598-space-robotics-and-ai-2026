#!/usr/bin/env python3
"""
TRN Node — Terrain-Relative Navigation scan-to-map matcher.

Builds a likelihood field from the drone's occupancy grid, then at 2 Hz
subsamples the rover's LiDAR scan and searches a (dx, dy, dyaw) candidate
grid to find the pose correction that maximises hit likelihood.  Publishes
corrected pose as PoseWithCovarianceStamped on /rover/trn_pose.

The node only corrects; it does not replace /rover/odom.  smart_rover_node
can optionally subscribe to /rover/trn_pose to seed the next A* re-plan.

Pipeline:
  map_cb  → build likelihood field + candidate array (once)
  odom_cb → dead-reckoning delta (every odom tick)
  lidar_cb → store latest scan (every scan tick)
  _update  → vectorized scoring, publish corrected pose (2 Hz)
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


class TRNNode(Node):
    # ── tuneable constants ─────────────────────────────────────────────────────
    N_RAYS = 36               # subsampled rays per scan update
    MAX_SCAN_R = 8.0          # metres — ignore rays beyond this
    MIN_SCAN_R = 0.25         # metres — ignore rays closer than this
    BLUR_SIGMA = 1.5          # cells — Gaussian blur width for likelihood field
    SEARCH_XY = 0.75          # metres each side in XY search
    SEARCH_XY_STEP = 0.25     # metres per XY step
    SEARCH_YAW = 0.087        # radians each side in yaw search (~5°, 1 step)
    SEARCH_YAW_STEP = 0.087   # radians per yaw step (~5°)
    MIN_VALID_RAYS = 8        # minimum rays needed to attempt a correction
    MIN_SCORE_IMPROVE = 0.05  # fractional improvement required to accept fix
    UPDATE_HZ = 2.0           # correction frequency

    def __init__(self):
        super().__init__('trn_node')

        gz_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/rover/trn_pose', 10)

        self.create_subscription(
            OccupancyGrid, '/drone/occupancy_grid', self.map_cb, map_qos)
        self.create_subscription(
            Odometry, '/rover/odom', self.odom_cb, gz_qos)
        self.create_subscription(
            LaserScan, '/rover/lidar', self.lidar_cb, gz_qos)

        # Likelihood field (built once from map)
        self.likelihood: np.ndarray | None = None
        self.grid_info = None

        # Dead-reckoning state
        self.trn_pos = np.array([0.0, 0.0])
        self.trn_yaw = 0.0
        self.trn_cov = 2.0        # scalar uncertainty proxy (metres)
        self._prev_odom_pos: np.ndarray | None = None
        self._prev_odom_yaw: float | None = None

        # Latest scan
        self.lidar_msg: LaserScan | None = None

        # Candidate offset array — built once after map arrives
        self.candidate_offsets: np.ndarray | None = None  # shape (K, 3)
        self._zero_idx: int = 0

        self._map_ready = False
        self._update_count = 0

        self.create_timer(1.0 / self.UPDATE_HZ, self._update)
        self.get_logger().info('TRNNode initialized — waiting for occupancy grid')

    # ── map callback ───────────────────────────────────────────────────────────

    def map_cb(self, msg: OccupancyGrid):
        if self._map_ready:
            return  # only process first map

        self.grid_info = msg.info
        w, h = msg.info.width, msg.info.height
        raw = np.array(msg.data, dtype=np.float32).reshape(h, w)

        # Convert occupancy to obstacle probability in [0, 1]
        occ_prob = np.zeros((h, w), dtype=np.float32)
        occ_prob[raw > 20] = 1.0           # occupied → 1
        occ_prob[raw == 0] = 0.0           # free     → 0
        occ_prob[raw < 0] = 0.3            # unknown  → low prior

        # Gaussian blur → likelihood field: nearby cells also score well
        blurred = self._gaussian_blur(occ_prob, self.BLUR_SIGMA)
        self.likelihood = np.clip(blurred, 0.0, 1.0)

        # Build candidate offset array (dx, dy, dyaw)
        offsets = []
        steps = np.arange(-self.SEARCH_XY, self.SEARCH_XY + 1e-9,
                          self.SEARCH_XY_STEP)
        yaw_steps = np.arange(-self.SEARCH_YAW, self.SEARCH_YAW + 1e-9,
                               self.SEARCH_YAW_STEP)
        for dy in steps:
            for dx in steps:
                for dyaw in yaw_steps:
                    offsets.append([float(dx), float(dy), float(dyaw)])
        self.candidate_offsets = np.array(offsets, dtype=np.float32)

        # Index of (0,0,0) candidate — used to compute baseline score
        dists = np.sum(self.candidate_offsets ** 2, axis=1)
        self._zero_idx = int(np.argmin(dists))

        self._map_ready = True
        self.get_logger().info(
            f'TRN likelihood field built: {w}x{h}, '
            f'{len(offsets)} candidates, blur_sigma={self.BLUR_SIGMA}')

    # ── odom callback — dead-reckoning prediction ──────────────────────────────

    def odom_cb(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if self._prev_odom_pos is None:
            self._prev_odom_pos = np.array([px, py])
            self._prev_odom_yaw = yaw
            self.trn_pos[:] = [px, py]
            self.trn_yaw = yaw
            return

        # Incremental delta in world frame → apply to TRN estimate
        dx = px - self._prev_odom_pos[0]
        dy = py - self._prev_odom_pos[1]
        dyaw = yaw - self._prev_odom_yaw
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi

        self.trn_pos[0] += dx
        self.trn_pos[1] += dy
        self.trn_yaw += dyaw
        while self.trn_yaw > math.pi:
            self.trn_yaw -= 2 * math.pi
        while self.trn_yaw < -math.pi:
            self.trn_yaw += 2 * math.pi

        self._prev_odom_pos[:] = [px, py]
        self._prev_odom_yaw = yaw

    # ── lidar callback — store latest scan ────────────────────────────────────

    def lidar_cb(self, msg: LaserScan):
        self.lidar_msg = msg

    # ── 2 Hz correction update ─────────────────────────────────────────────────

    def _update(self):
        if not self._map_ready:
            return
        if self.lidar_msg is None:
            return
        if self._prev_odom_pos is None:
            return

        scan = self.lidar_msg
        n = len(scan.ranges)
        if n == 0:
            return

        # Subsample N_RAYS uniformly
        indices = np.linspace(0, n - 1, self.N_RAYS, dtype=int)
        angles = (scan.angle_min
                  + indices * scan.angle_increment
                  + self.trn_yaw)       # world-frame ray angles
        ranges = np.array(scan.ranges, dtype=np.float32)[indices]

        # Filter valid ranges
        valid = ((ranges >= self.MIN_SCAN_R)
                 & (ranges <= min(self.MAX_SCAN_R, scan.range_max))
                 & np.isfinite(ranges))
        if int(valid.sum()) < self.MIN_VALID_RAYS:
            self._publish_trn()
            return

        scan_a = angles[valid]     # (M,)
        scan_r = ranges[valid]     # (M,)
        M = len(scan_a)

        scores = self._score_candidates(scan_a, scan_r, M)
        best_idx = int(np.argmax(scores))
        baseline_score = float(scores[self._zero_idx])
        best_score = float(scores[best_idx])

        if M > 0 and baseline_score > 0:
            relative_improve = (best_score - baseline_score) / (baseline_score + 1e-9)
        else:
            relative_improve = 0.0

        if relative_improve > self.MIN_SCORE_IMPROVE:
            dx, dy, dyaw = self.candidate_offsets[best_idx]
            self.trn_pos[0] += float(dx)
            self.trn_pos[1] += float(dy)
            self.trn_yaw += float(dyaw)
            while self.trn_yaw > math.pi:
                self.trn_yaw -= 2 * math.pi
            while self.trn_yaw < -math.pi:
                self.trn_yaw += 2 * math.pi
            self.trn_cov = max(0.05, self.trn_cov * 0.8)
            self._update_count += 1
            if self._update_count % 10 == 1:
                self.get_logger().info(
                    f'TRN fix #{self._update_count}: '
                    f'pos=({self.trn_pos[0]:.2f},{self.trn_pos[1]:.2f}) '
                    f'yaw={math.degrees(self.trn_yaw):.1f}° '
                    f'improve={relative_improve:.3f} cov={self.trn_cov:.3f}')
        else:
            self.trn_cov = min(2.0, self.trn_cov * 1.02)

        self._publish_trn()

    def _score_candidates(self, scan_a: np.ndarray, scan_r: np.ndarray,
                          M: int) -> np.ndarray:
        K = len(self.candidate_offsets)
        info = self.grid_info
        ox = info.origin.position.x
        oy = info.origin.position.y
        res = info.resolution
        H, W = self.likelihood.shape

        cx = self.trn_pos[0] + self.candidate_offsets[:, 0]    # (K,)
        cy = self.trn_pos[1] + self.candidate_offsets[:, 1]    # (K,)
        cyaw = self.trn_yaw + self.candidate_offsets[:, 2]     # (K,)

        # World-frame hit positions: (K, M)
        world_a = cyaw[:, np.newaxis] + scan_a[np.newaxis, :]
        hit_x = cx[:, np.newaxis] + scan_r[np.newaxis, :] * np.cos(world_a)
        hit_y = cy[:, np.newaxis] + scan_r[np.newaxis, :] * np.sin(world_a)

        # Grid indices
        gx = ((hit_x - ox) / res).astype(np.int32)
        gy = ((hit_y - oy) / res).astype(np.int32)

        in_bounds = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
        gx_safe = np.clip(gx, 0, W - 1)
        gy_safe = np.clip(gy, 0, H - 1)

        lhood = self.likelihood[gy_safe, gx_safe]
        lhood[~in_bounds] = 0.0

        return lhood.sum(axis=1)   # (K,)

    # ── publisher ─────────────────────────────────────────────────────────────

    def _publish_trn(self):
        msg = PoseWithCovarianceStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        msg.pose.pose.position.x = float(self.trn_pos[0])
        msg.pose.pose.position.y = float(self.trn_pos[1])
        msg.pose.pose.position.z = 0.0

        # Yaw → quaternion (z-axis rotation)
        half = self.trn_yaw / 2.0
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)

        c = float(self.trn_cov ** 2)
        cov = [0.0] * 36
        cov[0] = c    # x
        cov[7] = c    # y
        cov[35] = c * 0.1  # yaw (row 5, col 5)
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)

    # ── Gaussian blur (no scipy needed) ───────────────────────────────────────

    @staticmethod
    def _gaussian_blur(arr: np.ndarray, sigma: float) -> np.ndarray:
        k = max(1, int(3 * sigma))
        x = np.arange(-k, k + 1, dtype=np.float32)
        kernel = np.exp(-0.5 * (x / sigma) ** 2)
        kernel /= kernel.sum()
        result = arr.astype(np.float32).copy()
        # Row-wise pass
        for row in range(result.shape[0]):
            result[row] = np.convolve(result[row], kernel, mode='same')
        # Column-wise pass
        for col in range(result.shape[1]):
            result[:, col] = np.convolve(result[:, col], kernel, mode='same')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TRNNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
