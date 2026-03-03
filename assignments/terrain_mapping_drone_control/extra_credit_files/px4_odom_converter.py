#!/usr/bin/env python3
"""
PX4 Odometry → nav_msgs/Odometry converter for RTAB-Map.

Key design decisions (from 4 sessions of debugging):
1. Uses camera_info timestamp for odom publishing → guarantees approx_sync match
2. Converts PX4 NED/FRD → ROS ENU/FLU convention
3. Publishes TF (odom → base_link) so RTAB-Map has a complete TF tree
4. Uses BEST_EFFORT QoS to match PX4 topics (RELIABLE + BEST_EFFORT = zero messages)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class PX4OdomConverter(Node):
    def __init__(self):
        super().__init__('px4_odom_converter')

        # QoS for PX4 topics (MUST be BEST_EFFORT)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # QoS for publishing odom (BEST_EFFORT to avoid mismatch with any subscriber)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Latest camera timestamp — used to sync odom with camera frames
        self.latest_cam_stamp = None

        # Latest PX4 odom data
        self.latest_px4_odom = None

        # Subscribe to PX4 vehicle odometry
        self.px4_odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.px4_odom_cb,
            px4_qos
        )

        # Subscribe to camera info for timestamp synchronization
        self.caminfo_sub = self.create_subscription(
            CameraInfo,
            '/drone/front_depth/camera_info',
            self.caminfo_cb,
            10
        )

        # Publisher: nav_msgs/Odometry for RTAB-Map
        self.odom_pub = self.create_publisher(Odometry, '/rtabmap/odom', pub_qos)

        # TF broadcaster: odom → base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish at 30 Hz using latest data
        self.timer = self.create_timer(1.0 / 30.0, self.publish_odom)

        self.msg_count = 0
        self.get_logger().info('PX4 Odom Converter started. Waiting for camera + odom data...')

    def caminfo_cb(self, msg):
        """Cache the latest camera timestamp."""
        self.latest_cam_stamp = msg.header.stamp

    def px4_odom_cb(self, msg):
        """Cache the latest PX4 odometry."""
        self.latest_px4_odom = msg

    def publish_odom(self):
        """Publish nav_msgs/Odometry and TF using camera timestamp."""
        if self.latest_cam_stamp is None or self.latest_px4_odom is None:
            return

        msg = self.latest_px4_odom
        stamp = self.latest_cam_stamp

        # === Convert PX4 NED/FRD → ROS ENU/FLU ===
        # Position: NED → ENU
        x_enu = float(msg.position[1])    # East = y_ned
        y_enu = float(msg.position[0])    # North = x_ned
        z_enu = float(-msg.position[2])   # Up = -z_ned

        # Quaternion: NED/FRD → ENU/FLU
        # PX4 VehicleOdometry.q = [w, x, y, z]
        qw = float(msg.q[0])
        qx = float(msg.q[2])    # swap x↔y
        qy = float(msg.q[1])
        qz = float(-msg.q[3])   # negate z

        # === Build nav_msgs/Odometry ===
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x_enu
        odom.pose.pose.position.y = y_enu
        odom.pose.pose.position.z = z_enu

        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz

        # Velocity (NED → ENU)
        if msg.velocity[0] != 0.0 or msg.velocity[1] != 0.0 or msg.velocity[2] != 0.0:
            odom.twist.twist.linear.x = float(msg.velocity[1])
            odom.twist.twist.linear.y = float(msg.velocity[0])
            odom.twist.twist.linear.z = float(-msg.velocity[2])

        self.odom_pub.publish(odom)

        # === Publish TF: odom → base_link ===
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x_enu
        t.transform.translation.y = y_enu
        t.transform.translation.z = z_enu
        t.transform.rotation.w = qw
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        self.tf_broadcaster.sendTransform(t)

        self.msg_count += 1
        if self.msg_count % 150 == 1:  # Log every ~5 seconds
            self.get_logger().info(
                f'Odom published #{self.msg_count}: pos=({x_enu:.2f}, {y_enu:.2f}, {z_enu:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomConverter()
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
