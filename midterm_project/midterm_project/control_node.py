#!/usr/bin/env python3
"""
Phase 51-59: Control Node — Full State Machine
States: WAIT_ODOM -> PREFLIGHT -> ARMING -> TAKEOFF -> SURVEY -> HOVER

BUG FIXES vs previous version:
1. target_z computed from home_position (not hardcoded -5.0 absolute NED)
2. Arm command retried every 0.5s until liftoff detected (PX4 may reject first attempt)
3. Mode switch sent separately from arm, with counter to ensure offboard confirmed first
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

        # QoS — must match PX4 exactly
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

        # Subscribers — no VehicleStatus (silent type mismatch bug)
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_callback, qos)

        # State
        self.state           = self.WAIT_ODOM
        self.odom_received   = False
        self.position        = np.array([0.0, 0.0, 0.0])
        self.home_position   = None

        # FIX 1: target_z set from home_position, not hardcoded
        self.target_z        = None

        # Preflight
        self.preflight_count = 0

        # FIX 2+3: arm with retry every 10 ticks (0.5s), mode switch first
        self.arm_tick        = 0
        self.arm_retry_interval = 10  # ticks at 20Hz = every 0.5s

        # Survey
        self.waypoints       = []
        self.waypoint_idx    = 0

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f'ControlNode initialized — height={self.takeoff_height}m, '
            f'extent=±{self.survey_extent}m, spacing={self.survey_spacing}m')

    def odom_callback(self, msg: VehicleOdometry):
        self.position = np.array([
            msg.position[0],
            msg.position[1],
            msg.position[2]
        ])
        if not self.odom_received:
            self.odom_received   = True
            self.home_position   = self.position.copy()
            # FIX 1: compute absolute NED Z target from actual home altitude
            self.target_z        = float(self.home_position[2]) - self.takeoff_height
            self.get_logger().info(
                f'Odometry received — home={self.home_position}, '
                f'target_z={self.target_z:.3f} (NED)')

    # ------------------------------------------------------------------
    # Altitude helper — always relative to home, positive = up
    # ------------------------------------------------------------------
    def _alt(self):
        return -(self.position[2] - self.home_position[2])

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
        # Send offboard heartbeat continuously before arming
        # PX4 requires ~1s of offboard setpoints before it accepts mode switch
        self._publish_offboard_mode()
        self._publish_setpoint(0.0, 0.0, self.target_z)
        self.preflight_count += 1
        if self.preflight_count % 20 == 0:
            self.get_logger().info(
                f'PREFLIGHT: {self.preflight_count} setpoints sent')
        if self.preflight_count >= 40:
            self.get_logger().info('PREFLIGHT done — ARMING')
            self.state = self.ARMING

    def _state_arming(self):
        # Always keep publishing offboard heartbeat
        self._publish_offboard_mode()
        self._publish_setpoint(0.0, 0.0, self.target_z)

        self.arm_tick += 1

        # FIX 3: Send mode switch first, arm on next retry interval
        # This gives PX4 time to confirm offboard before arming
        if self.arm_tick % self.arm_retry_interval == 1:
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info(
                f'Mode switch sent (tick {self.arm_tick}), alt={self._alt():.2f}m')

        # FIX 2: Retry arm command every interval until liftoff
        if self.arm_tick % self.arm_retry_interval == 2:
            self._send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info(
                f'Arm command sent (tick {self.arm_tick}), alt={self._alt():.2f}m')

        # Detect liftoff via altitude change > 1.0m relative to home
        if self._alt() > 1.0:
            self.get_logger().info(
                f'Liftoff detected at alt={self._alt():.2f}m — TAKEOFF')
            self.state = self.TAKEOFF

    def _state_takeoff(self):
        self._publish_offboard_mode()
        self._publish_setpoint(0.0, 0.0, self.target_z)
        alt = self._alt()
        if abs(alt - self.takeoff_height) < 0.5:
            self.get_logger().info(
                f'Takeoff complete at {alt:.2f}m — generating survey pattern')
            self._generate_survey_pattern()
            self.state = self.SURVEY

    def _state_survey(self):
        self._publish_offboard_mode()
        if self.waypoint_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints complete — HOVER')
            self.state = self.HOVER
            return

        wp = self.waypoints[self.waypoint_idx]
        self._publish_setpoint(wp[0], wp[1], self.target_z)

        dist = np.linalg.norm(self.position[:2] - np.array([wp[0], wp[1]]))
        if dist < self.waypoint_threshold:
            self.get_logger().info(
                f'Waypoint {self.waypoint_idx + 1}/{len(self.waypoints)} '
                f'reached ({wp[0]:.0f},{wp[1]:.0f}) — dist={dist:.2f}m')
            self.waypoint_idx += 1

    def _state_hover(self):
        self._publish_offboard_mode()
        self._publish_setpoint(
            self.position[0], self.position[1], self.target_z)

    # ------------------------------------------------------------------
    # Survey pattern — lawnmower grid
    # Gentle turns: waypoints alternate row ends, no sharp reversals
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
        self.waypoints    = waypoints
        self.waypoint_idx = 0
        self.get_logger().info(
            f'Survey pattern: {len(waypoints)} waypoints, '
            f'extent=±{self.survey_extent}m, spacing={self.survey_spacing}m')

    # ------------------------------------------------------------------
    # PX4 helpers
    # ------------------------------------------------------------------
    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    def _publish_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position  = [float(x), float(y), float(z)]
        msg.yaw       = -math.pi / 2.0
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.setpoint_pub.publish(msg)

    def _send_command(self, command: int, param1: float = 0.0,
                      param2: float = 0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self.get_clock().now().nanoseconds // 1000
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
