#!/usr/bin/env python3
"""
PX4 Odometry -> nav_msgs/Odometry converter for RTAB-Map.

Lessons applied from 5 debugging sessions:
- Uses camera_info timestamp (not get_clock().now() which is 0, not PX4 timestamp which is incompatible)
- Publishes BEST_EFFORT QoS (RELIABLE + BEST_EFFORT subscriber = zero messages)
- Broadcasts TF odom->base_link (RTAB-Map needs this for depth projection)
- Timer-based: publishes latest data at 30Hz, only when both camera and PX4 data available
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

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.latest_cam_stamp = None
        self.latest_px4_odom = None

        self.px4_odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.px4_odom_cb, px4_qos)

        self.caminfo_sub = self.create_subscription(
            CameraInfo, '/drone/front_depth/camera_info',
            self.caminfo_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, '/rtabmap/odom', pub_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / 30.0, self.publish_odom)
        self.msg_count = 0
        self.get_logger().info('PX4 Odom Converter started. Waiting for camera + PX4 data...')

    def caminfo_cb(self, msg):
        self.latest_cam_stamp = msg.header.stamp

    def px4_odom_cb(self, msg):
        self.latest_px4_odom = msg

    def publish_odom(self):
        if self.latest_cam_stamp is None or self.latest_px4_odom is None:
            return

        msg = self.latest_px4_odom
        stamp = self.latest_cam_stamp

        # NED -> ENU position
        x = float(msg.position[1])
        y = float(msg.position[0])
        z = float(-msg.position[2])

        # NED/FRD -> ENU/FLU quaternion (PX4 q = [w, x, y, z])
        qw = float(msg.q[0])
        qx = float(msg.q[2])
        qy = float(msg.q[1])
        qz = float(-msg.q[3])

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz

        if msg.velocity[0] != 0.0 or msg.velocity[1] != 0.0 or msg.velocity[2] != 0.0:
            odom.twist.twist.linear.x = float(msg.velocity[1])
            odom.twist.twist.linear.y = float(msg.velocity[0])
            odom.twist.twist.linear.z = float(-msg.velocity[2])

        pose_cov = [0.0] * 36
        pose_cov[0] = pose_cov[7] = pose_cov[14] = 0.01
        pose_cov[21] = pose_cov[28] = pose_cov[35] = 0.001
        odom.pose.covariance = pose_cov
        odom.twist.covariance = pose_cov

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w = qw
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        self.tf_broadcaster.sendTransform(t)

        self.msg_count += 1
        if self.msg_count % 150 == 1:
            self.get_logger().info(
                f'Odom #{self.msg_count}: pos=({x:.2f}, {y:.2f}, {z:.2f}) stamp=({stamp.sec}.{stamp.nanosec})')


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
