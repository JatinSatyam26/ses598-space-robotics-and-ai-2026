#!/usr/bin/env python3
"""
flight_control_node.py — SES 598 Live Demo
Physics-based drone flight using MulticopterVelocityControl.

Publishes geometry_msgs/Twist to /drone/cmd_vel
  (bridged to Gazebo flying_drone/gazebo/command/twist)

Subscribes to nav_msgs/Odometry from /drone/odom
  (bridged from Gazebo model/flying_drone/odometry)

State machine:
  TAKEOFF  → climb to 4.5 m
  SURVEY   → lawnmower waypoints at 5 m
  HOVER    → hold for 3 s then land
  LAND     → descend to 0.5 m
  DONE     → publish zero twist, print mission complete
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ── Mission waypoints ────────────────────────────────────────────────────────
WAYPOINTS = [
    (-10.0, -10.0, 5.0),
    ( 10.0, -10.0, 5.0),
    ( 10.0,  -7.0, 5.0),
    (-10.0,  -7.0, 5.0),
    (-10.0,  -4.0, 5.0),
    ( 10.0,  -4.0, 5.0),
    ( 10.0,  -1.0, 5.0),
    (-10.0,  -1.0, 5.0),
    (-10.0,   2.0, 5.0),
    ( 10.0,   2.0, 5.0),
]

# ── Controller parameters ────────────────────────────────────────────────────
KP            = 0.5     # proportional gain
VXY_MAX       = 1.5     # m/s horizontal clamp
VZ_MAX        = 1.0     # m/s vertical clamp
WP_REACH_DIST = 1.5     # m — waypoint reached threshold
TAKEOFF_ALT   = 4.5     # m — consider takeoff complete above this altitude
LAND_ALT      = 0.5     # m — consider landing complete below this altitude
HOVER_SECS    = 3.0     # s — hover duration before landing

# ── QoS ─────────────────────────────────────────────────────────────────────
_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FlightControlNode(Node):

    def __init__(self):
        super().__init__('flight_control_node')

        self.cmd_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/drone/odom', self._odom_cb, _QOS)

        self.pos = None        # (x, y, z) from odometry
        self.state = 'WAIT_ODOM'
        self.wp_idx = 0
        self.hover_start = None

        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info('FlightControlNode started — waiting for odometry…')

    # ────────────────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)
        if self.state == 'WAIT_ODOM':
            self.state = 'TAKEOFF'
            self.get_logger().info(
                f'Odometry received — home ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}). '
                'TAKEOFF starting.')

    # ────────────────────────────────────────────────────────────────────────
    def _tick(self):
        if self.state == 'WAIT_ODOM':
            return                           # nothing to do yet

        twist = Twist()

        if self.state == 'TAKEOFF':
            if self.pos and self.pos[2] < TAKEOFF_ALT:
                twist.linear.z = 2.0        # climb at 2 m/s
            else:
                self.get_logger().info(
                    f'TAKEOFF complete at z={self.pos[2]:.2f} m. Starting SURVEY.')
                self.state = 'SURVEY'

        elif self.state == 'SURVEY':
            if self.wp_idx >= len(WAYPOINTS):
                self.get_logger().info('All waypoints visited — entering HOVER.')
                self.state = 'HOVER'
                self.hover_start = self.get_clock().now()
            else:
                wx, wy, wz = WAYPOINTS[self.wp_idx]
                cx, cy, cz = self.pos
                dx, dy, dz = wx - cx, wy - cy, wz - cz
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                if dist < WP_REACH_DIST:
                    self.get_logger().info(
                        f'Waypoint {self.wp_idx+1}/{len(WAYPOINTS)} reached '
                        f'({wx:.1f}, {wy:.1f}, {wz:.1f})')
                    self.wp_idx += 1
                else:
                    twist.linear.x = _clamp(KP * dx, -VXY_MAX, VXY_MAX)
                    twist.linear.y = _clamp(KP * dy, -VXY_MAX, VXY_MAX)
                    twist.linear.z = _clamp(KP * dz, -VZ_MAX,  VZ_MAX)

        elif self.state == 'HOVER':
            elapsed = (self.get_clock().now() - self.hover_start).nanoseconds * 1e-9
            if elapsed >= HOVER_SECS:
                self.get_logger().info('HOVER done — LAND starting.')
                self.state = 'LAND'

        elif self.state == 'LAND':
            if self.pos and self.pos[2] > LAND_ALT:
                twist.linear.z = -1.0       # descend at 1 m/s
            else:
                self.get_logger().info('LAND complete — mission DONE.')
                self.state = 'DONE'

        elif self.state == 'DONE':
            self.cmd_pub.publish(Twist())    # zero twist → hover
            self.get_logger().info('Mission complete — shutting down.')
            rclpy.shutdown()
            return

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FlightControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())    # stop motors on exit
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
