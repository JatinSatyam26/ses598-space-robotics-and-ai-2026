#!/usr/bin/env python3
"""
Phase 71-80: Perception Node — MASt3R Dense Reconstruction as ROS2 Node
- Subscribes to /drone/front_rgb, /drone/front_depth, /fmu/out/vehicle_odometry
- Keyframe selection: distance threshold 1.0m + angle threshold 15deg
- Lazy model loading (MASt3R loaded on first image received)
- Threaded MASt3R inference (GPU-bound, must not block ROS callbacks)
- NED->world frame transformation using quaternion rotation
- Publishes PointCloud2 with XYZ + RGB + confidence fields
- Publishes status at 1 Hz
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry

import numpy as np
import threading
import time
import os
import sys
from cv_bridge import CvBridge


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        # Parameters
        self.declare_parameter('keyframe_distance', 1.0)
        self.declare_parameter('keyframe_angle_deg', 15.0)
        self.declare_parameter('model_path', '')
        self.declare_parameter('min_confidence', 1.5)

        self.keyframe_distance  = self.get_parameter('keyframe_distance').value
        self.keyframe_angle_deg = self.get_parameter('keyframe_angle_deg').value
        self.model_path         = self.get_parameter('model_path').value
        self.min_confidence     = self.get_parameter('min_confidence').value

        # Default model path if not set
        if not self.model_path:
            self.model_path = os.path.expanduser(
                '~/ses598-space-robotics-and-ai-2026/midterm_and_final_project/deps/'
                'mast3r/checkpoints/'
                'MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth')

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
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_callback, qos_px4)

        # Publishers
        self.cloud_pub  = self.create_publisher(PointCloud2, '/midterm/pointcloud', 10)
        self.status_pub = self.create_publisher(String, '/midterm/perception_status', 10)

        # State
        self.bridge          = CvBridge()
        self.model           = None
        self.model_loaded    = False
        self.model_loading   = False

        # Odometry
        self.position        = np.array([0.0, 0.0, 0.0])
        self.quaternion      = np.array([1.0, 0.0, 0.0, 0.0])  # w,x,y,z

        # Keyframes
        self.keyframes       = []   # list of {'image': np, 'position': np, 'quaternion': np}
        self.last_kf_pos     = None
        self.last_kf_yaw     = None

        # Inference queue
        self.inference_queue = []
        self.inference_lock  = threading.Lock()
        self.inference_thread = threading.Thread(
            target=self._inference_worker, daemon=True)
        self.inference_thread.start()

        # Accumulated point cloud
        self.cloud_points    = []   # list of (x, y, z, r, g, b, conf)

        # Status timer at 1 Hz
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f'PerceptionNode initialized — '
            f'keyframe_distance={self.keyframe_distance}m, '
            f'keyframe_angle={self.keyframe_angle_deg}deg')
        self.get_logger().info(f'Model path: {self.model_path}')

    # ------------------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------------------
    def odom_callback(self, msg: VehicleOdometry):
        self.position   = np.array([msg.position[0],
                                    msg.position[1],
                                    msg.position[2]])
        self.quaternion = np.array([msg.q[0], msg.q[1],
                                    msg.q[2], msg.q[3]])

    # ------------------------------------------------------------------
    # RGB callback — keyframe selection
    # ------------------------------------------------------------------
    def rgb_callback(self, msg: Image):
        # Lazy model loading on first image
        if not self.model_loaded and not self.model_loading:
            self.model_loading = True
            t = threading.Thread(target=self._load_model, daemon=True)
            t.start()
            return

        if not self.model_loaded:
            return

        # Convert image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return

        # Keyframe selection
        pos = self.position.copy()
        yaw = self._yaw_from_quaternion(self.quaternion)

        if self.last_kf_pos is None:
            self._add_keyframe(frame, pos, yaw)
            return

        dist = np.linalg.norm(pos - self.last_kf_pos)
        angle_diff = abs(yaw - self.last_kf_yaw)
        angle_diff = min(angle_diff, 2 * np.pi - angle_diff)

        if (dist >= self.keyframe_distance or
                np.degrees(angle_diff) >= self.keyframe_angle_deg):
            self._add_keyframe(frame, pos, yaw)

    def _add_keyframe(self, frame, pos, yaw):
        kf = {'image': frame, 'position': pos.copy(), 'quaternion': self.quaternion.copy()}
        self.keyframes.append(kf)
        self.last_kf_pos = pos.copy()
        self.last_kf_yaw = yaw

        # Queue inference if we have at least 2 keyframes
        if len(self.keyframes) >= 2:
            with self.inference_lock:
                self.inference_queue.append(
                    (self.keyframes[-2].copy(), self.keyframes[-1].copy()))

        self.get_logger().info(
            f'Keyframe {len(self.keyframes)} added at '
            f'pos=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})')

    # ------------------------------------------------------------------
    # Lazy model loading
    # ------------------------------------------------------------------
    def _load_model(self):
        try:
            self.get_logger().info('Loading MASt3R model...')
            deps_path = os.path.expanduser(
                '~/ses598-space-robotics-and-ai-2026/midterm_and_final_project/deps/mast3r')
            sys.path.insert(0, deps_path)
            sys.path.insert(0, os.path.join(deps_path, 'dust3r'))

            import torch
            from mast3r.model import AsymmetricMASt3R

            self.model = AsymmetricMASt3R.from_pretrained(
                self.model_path).cuda().eval()

            vram = torch.cuda.memory_allocated() / 1e9
            self.get_logger().info(f'MASt3R loaded — VRAM: {vram:.2f} GB')
            self.model_loaded  = True
            self.model_loading = False

        except Exception as e:
            self.get_logger().error(f'Model loading failed: {e}')
            self.model_loading = False

    # ------------------------------------------------------------------
    # Inference worker thread
    # ------------------------------------------------------------------
    def _inference_worker(self):
        while True:
            pair = None
            with self.inference_lock:
                if self.inference_queue:
                    pair = self.inference_queue.pop(0)

            if pair is None:
                time.sleep(0.1)
                continue

            if not self.model_loaded:
                time.sleep(0.5)
                continue

            try:
                self._run_inference(pair)
            except Exception as e:
                self.get_logger().error(f'Inference failed: {e}')

    def _run_inference(self, pair):
        import torch
        from mast3r.fast_nn import fast_reciprocal_NNs
        from dust3r.inference import inference
        from dust3r.utils.image import load_images
        import tempfile
        import cv2

        kf1, kf2 = pair
        t0 = time.time()

        # Save frames to temp files for MASt3R
        with tempfile.TemporaryDirectory() as tmpdir:
            p1 = os.path.join(tmpdir, 'img1.png')
            p2 = os.path.join(tmpdir, 'img2.png')
            cv2.imwrite(p1, cv2.cvtColor(kf1['image'], cv2.COLOR_RGB2BGR))
            cv2.imwrite(p2, cv2.cvtColor(kf2['image'], cv2.COLOR_RGB2BGR))

            images = load_images([p1, p2], size=512)
            output = inference([tuple(images)], self.model,
                               'cuda', batch_size=1, verbose=False)

        t1 = time.time()

        # Extract points and confidence
        pts3d = output['pred1']['pts3d'][0].detach().cpu().numpy()
        conf  = output['pred1']['conf'][0].detach().cpu().numpy()
        rgb   = kf1['image']

        # Flatten
        h, w  = pts3d.shape[:2]
        pts   = pts3d.reshape(-1, 3)
        confs = conf.reshape(-1)

        # Resize RGB to match
        import cv2
        rgb_resized = cv2.resize(rgb, (w, h))
        colors = rgb_resized.reshape(-1, 3)

        # Filter by confidence
        mask = confs > self.min_confidence
        pts   = pts[mask]
        colors = colors[mask]
        confs  = confs[mask]

        # Transform to world frame using first keyframe pose
        pts_world = self._ned_to_world(pts, kf1['position'], kf1['quaternion'])

        # Accumulate
        for i in range(len(pts_world)):
            self.cloud_points.append((
                pts_world[i, 0], pts_world[i, 1], pts_world[i, 2],
                int(colors[i, 0]), int(colors[i, 1]), int(colors[i, 2]),
                float(confs[i])
            ))

        self.get_logger().info(
            f'Inference done: {len(pts_world)} pts, '
            f'{t1-t0:.2f}s, total={len(self.cloud_points)}')

        self._publish_cloud()

    # ------------------------------------------------------------------
    # NED -> world frame transformation
    # ------------------------------------------------------------------
    def _ned_to_world(self, pts, position, quaternion):
        w, x, y, z = quaternion
        R = np.array([
            [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
            [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
            [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])
        return (R @ pts.T).T + position

    def _yaw_from_quaternion(self, q):
        w, x, y, z = q
        return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    # ------------------------------------------------------------------
    # PointCloud2 publisher
    # ------------------------------------------------------------------
    def _publish_cloud(self):
        if not self.cloud_points:
            return

        pts = np.array(self.cloud_points, dtype=np.float32)
        msg = PointCloud2()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.height          = 1
        msg.width           = len(pts)
        msg.is_dense        = False
        msg.is_bigendian    = False

        fields = [
            PointField(name='x',    offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',    offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',    offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='r',    offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='g',    offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='b',    offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='conf', offset=24, datatype=PointField.FLOAT32, count=1),
        ]
        msg.fields     = fields
        msg.point_step = 28
        msg.row_step   = msg.point_step * msg.width
        msg.data       = pts.tobytes()
        self.cloud_pub.publish(msg)

    def publish_status(self):
        msg = String()
        msg.data = (
            f'keyframes={len(self.keyframes)}, '
            f'cloud_pts={len(self.cloud_points)}, '
            f'model_loaded={self.model_loaded}, '
            f'queue={len(self.inference_queue)}'
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
