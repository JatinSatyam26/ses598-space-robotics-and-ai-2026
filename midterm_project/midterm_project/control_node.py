#!/usr/bin/env python3
"""
Phase 51-59: Control Node — Full State Machine
States: WAIT_ODOM -> PREFLIGHT -> ARMING -> TAKEOFF -> SURVEY -> HOVER

KEY LESSONS FROM TRANSCRIPT:
- Remove VehicleStatus dependency (silent type mismatch causes stuck INIT)
- Detect arming via altitude change > 1.0m, not status flag
- 5m is the proven stable height (15m hits SITL ceiling, 1m triggers ground effect)
- Send 40+ preflight setpoints before arming
- Survey turn speed must be gentle to avoid attitude roll failsafe
- Waypoint threshold: 1.5m
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)

import numpy as np
import math


class ControlNode(Node):

    WAIT_ODOM = 'WAIT_ODOM'
    PREFLIGHT  = 'PREFLIGHT'
    ARMING     = 'ARMING'
    TAKEOFF    = 'TAKEOFF'
    SURVEY     = 'SURVEY'
    HOVER      = 'HOVER'

    def __init__(self):
        super().__init__('control_node')

        # Parameters
        self.declare_parameter('takeoff_height', 5.0)
        self.declare_parameter('survey_extent',  15.0)
        self.declare_parameter('survey_spacing',  5.0)
        self.declare_parameter('waypoint_threshold', 1.5)
        self.declare_parameter('survey_speed', 2.0)

        self.takeoff_height     = self.get_parameter('takeoff_height').value
        self.survey_extent      = self.get_parameter('survey_extent').value
        self.survey_spacing     = self.get_parameter('survey_spacing').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.survey_speed       = self.get_parameter('survey_speed').value

        # QoS — LESSON: must match PX4 exactly
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # Subscribers — LESSON: no VehicleStatus, silent type mismatch
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_callback, qos)

        # State
        self.state           = self.WAIT_ODOM
        self.odom_received   = False
        self.position        = np.array([0.0, 0.0, 0.0])
        self.home_position   = None
        self.preflight_count = 0
        self.arm_attempts    = 0

        # Survey
        self.waypoints     = []
        self.waypoint_idx  = 0

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f'ControlNode initialized — height={self.takeoff_height}m, '
            f'extent=±{self.survey_extent}m, spacing={self.survey_spacing}m')

    def odom_callback(self, msg: VehicleOdometry):
        self.position = np.array([msg.position[0], msg.position[1], msg.position[2]])
        if not self.odom_received:
            self.odom_received = True
            self.home_position = self.position.copy()
            self.get_logger().info(f'Odometry received — home={self.home_position}')

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def control_loop(self):
        if   self.state == self.WAIT_ODOM: self._state_wait_odom()
        elif self.state == self.PREFLIGHT:  self._state_preflight()
        elif self.state == self.ARMING:     self._state_arming()
        elif self.state == self.TAKEOFF:    self._state_takeoff()
        elif self.state == self.SURVEY:     self._state_survey()
        elif self.state == self.HOVER:      self._state_hover()

    def _state_wait_odom(self):
        if self.odom_received:
            self.get_logger().info('Odometry confirmed — entering PREFLIGHT')
            self.state = self.PREFLIGHT

    def _state_preflight(self):
        self._publish_offboard_mode()
        self._publish_setpoint(0.0, 0.0, -self.takeoff_height)
        self.preflight_count += 1
        if self.preflight_count % 20 == 0:
            self.get_logger().info(f'PREFLIGHT: {self.preflight_count} setpoints sent')
        if self.preflight_count >= 40:
            self.get_logger().info('PREFLIGHT done — ARMING')
            self.state = self.ARMING

    def _state_arming(self):
        self._publish_offboard_mode()
        self._publish_setpoint(0.0, 0.0, -self.takeoff_height)

        alt = -self.position[2]   # NED: Z negative is up
        self.get_logger().info(f'Arm attempt — alt={alt:.1f}m')

        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.arm_attempts += 1

        # LESSON: detect arming via altitude change, not status flag
        if alt > 1.0:
            self.get_logger().info(f'Movement detected (alt={alt:.1f}m) — TAKEOFF')
            self.state = self.TAKEOFF

    def _state_takeoff(self):
        self._publish_offboard_mode()
        self._publish_setpoint(0.0, 0.0, -self.takeoff_height)
        alt = -self.position[2]
        if abs(alt - self.takeoff_height) < 0.5:
            self.get_logger().info(f'Takeoff complete at {alt:.1f}m — starting SURVEY')
            self._generate_survey_pattern()
            self.state = self.SURVEY

    def _state_survey(self):
        self._publish_offboard_mode()
        if self.waypoint_idx >= len(self.waypoints):
            self.get_logger().info('Survey complete — HOVER')
            self.state = self.HOVER
            return

        wp = self.waypoints[self.waypoint_idx]
        self._publish_setpoint(wp[0], wp[1], -self.takeoff_height)

        dist = np.linalg.norm(self.position[:2] - np.array([wp[0], wp[1]]))
        if dist < self.waypoint_threshold:
            self.get_logger().info(
                f'Waypoint {self.waypoint_idx + 1}/{len(self.waypoints)} '
                f'reached ({wp[0]:.0f},{wp[1]:.0f})')
            self.waypoint_idx += 1

    def _state_hover(self):
        self._publish_offboard_mode()
        self._publish_setpoint(
            self.position[0], self.position[1], -self.takeoff_height)

    # ------------------------------------------------------------------
    # Survey pattern — lawnmower grid
    # LESSON: gentle turns, avoid aggressive direction changes
    # ------------------------------------------------------------------
    def _generate_survey_pattern(self):
        waypoints = []
        x = -self.survey_extent
        row = 0
        while x <= self.survey_extent + 0.1:
            if row % 2 == 0:
                waypoints.append((x, -self.survey_extent))
                waypoints.append((x,  self.survey_extent))
            else:
                waypoints.append((x,  self.survey_extent))
                waypoints.append((x, -self.survey_extent))
            x += self.survey_spacing
            row += 1
        self.waypoints = waypoints
        self.waypoint_idx = 0
        self.get_logger().info(
            f'Survey pattern generated: {len(waypoints)} waypoints')

    # ------------------------------------------------------------------
    # PX4 helpers
    # ------------------------------------------------------------------
    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    def _publish_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = -math.pi / 2.0
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.setpoint_pub.publish(msg)

    def _send_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
