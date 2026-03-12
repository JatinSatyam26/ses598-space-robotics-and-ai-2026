#!/usr/bin/env python3
"""
Control Node — PX4 offboard interface for the midterm project.
Handles arming, takeoff, position commands, and landing.
Adapted from Assignment 3 cylinder_landing_node.py.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleOdometry,
    VehicleStatus
)
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import math


class ControlNode(Node):
    """Low-level PX4 offboard control node."""

    def __init__(self):
        super().__init__('control_node')

        # QoS profile for PX4 communication (same as Assignment 3)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # === PX4 Publishers ===
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # === PX4 Subscribers ===
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_callback, qos_profile)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.status_callback, qos_profile)

        # === Inter-node communication ===
        # Receives position commands from planning_node
        self.create_subscription(
            TrajectorySetpoint, '/midterm/trajectory_cmd',
            self.trajectory_cmd_callback, 10)
        # Receives high-level mission commands (TAKEOFF, EXPLORE, LAND)
        self.create_subscription(
            String, '/midterm/mission_command',
            self.mission_cmd_callback, 10)

        # Publishes current drone state for other nodes
        self.state_pub = self.create_publisher(
            String, '/midterm/drone_state', 10)
        self.odom_pub = self.create_publisher(
            VehicleOdometry, '/midterm/odom', 10)

        # === State variables ===
        self.vehicle_odom = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.current_trajectory_cmd = None

        # Mission parameters (NED frame: z is negative up)
        self.declare_parameter('takeoff_height', 10.0)
        self.declare_parameter('position_threshold', 0.3)
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.position_threshold = self.get_parameter('position_threshold').value

        # State machine
        self.state = 'IDLE'  # IDLE, ARMING, TAKEOFF, HOVER, ACTIVE, LANDING, LANDED

        # 10Hz control loop (same as Assignment 3)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Control node initialized')

    # === PX4 Callbacks ===
    def odom_callback(self, msg):
        self.vehicle_odom = msg
        self.odom_pub.publish(msg)

    def status_callback(self, msg):
        self.vehicle_status = msg

    # === Command Callbacks ===
    def trajectory_cmd_callback(self, msg):
        self.current_trajectory_cmd = msg

    def mission_cmd_callback(self, msg):
        cmd = msg.data.upper()
        self.get_logger().info(f'Mission command received: {cmd}')
        if cmd == 'TAKEOFF' and self.state == 'IDLE':
            self.state = 'ARMING'
        elif cmd == 'EXPLORE' and self.state == 'HOVER':
            self.state = 'ACTIVE'
        elif cmd == 'LAND':
            self.state = 'LANDING'

    # === PX4 Interface Methods (from Assignment 3) ===
    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode engaged')

    def land(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def get_position(self):
        """Return current position as (x, y, z) in NED."""
        p = self.vehicle_odom.position
        return (p[0], p[1], p[2])

    def get_altitude(self):
        """Return current altitude in meters (positive up)."""
        return -self.vehicle_odom.position[2]

    # === Main Control Loop ===
    def control_loop(self):
        self.publish_offboard_control_mode()

        if self.state == 'IDLE':
            self.publish_setpoint(0.0, 0.0, 0.0)

        elif self.state == 'ARMING':
            if self.offboard_setpoint_counter < 10:
                self.publish_setpoint(0.0, 0.0, 0.0)
                self.offboard_setpoint_counter += 1
            elif self.offboard_setpoint_counter == 10:
                self.engage_offboard_mode()
                self.arm()
                self.offboard_setpoint_counter += 1
            else:
                self.state = 'TAKEOFF'
                self.get_logger().info('Armed — transitioning to TAKEOFF')

        elif self.state == 'TAKEOFF':
            target_z = -self.takeoff_height  # NED: negative is up
            self.publish_setpoint(0.0, 0.0, target_z)
            if self.get_altitude() > (self.takeoff_height - self.position_threshold):
                self.state = 'HOVER'
                self.get_logger().info(
                    f'Reached {self.get_altitude():.1f}m — HOVER state')

        elif self.state == 'HOVER':
            target_z = -self.takeoff_height
            self.publish_setpoint(0.0, 0.0, target_z)

        elif self.state == 'ACTIVE':
            # Forward trajectory commands from planning_node
            if self.current_trajectory_cmd is not None:
                self.trajectory_pub.publish(self.current_trajectory_cmd)
            else:
                target_z = -self.takeoff_height
                self.publish_setpoint(0.0, 0.0, target_z)

        elif self.state == 'LANDING':
            self.land()
            self.state = 'LANDED'

        elif self.state == 'LANDED':
            pass

        # Publish current state for other nodes
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
