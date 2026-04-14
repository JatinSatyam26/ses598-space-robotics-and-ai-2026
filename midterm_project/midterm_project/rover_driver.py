#!/usr/bin/env python3
"""
rover_driver.py — SES 598 Live Demo
Drives the Mars rover along the pre-planned A* path using DiffDrive.

Reads:   ~/midterm_results/rover_path.json  (list of {x, y} waypoints)
Subs:    /rover/odom  (nav_msgs/Odometry)
Pubs:    /rover/cmd_vel  (geometry_msgs/Twist)

Proportional controller:
  angular.z  = Kp_yaw * angle_error   (turn toward waypoint)
  linear.x   = 0.3 m/s when |angle_error| < 0.3 rad, else 0.0

On completion:
  saves actual traversed path to ~/midterm_results/rover_actual_path.json
  prints "Navigation complete"
"""

import json
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ── Parameters ───────────────────────────────────────────────────────────────
KP_YAW        = 2.0    # angular gain
FORWARD_SPEED = 0.3    # m/s
ANGLE_OK      = 0.3    # rad — drive forward when pointing this close
WP_REACH      = 0.3    # m — waypoint reached threshold (2-D)
RESULTS_DIR   = os.path.expanduser('~/midterm_results')

_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _yaw_from_quat(qx, qy, qz, qw):
    """Extract yaw from quaternion."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def _angle_diff(a, b):
    """Shortest signed angle difference a - b in [-pi, pi]."""
    d = a - b
    while d >  math.pi: d -= 2.0 * math.pi
    while d < -math.pi: d += 2.0 * math.pi
    return d


class RoverDriver(Node):

    def __init__(self):
        super().__init__('rover_driver')

        # Load path
        path_file = os.path.join(RESULTS_DIR, 'rover_path.json')
        with open(path_file) as f:
            data = json.load(f)
        raw = data['path'] if isinstance(data, dict) else data
        # Support both {'x':…,'y':…} and {'position':[x,y,z]} formats
        self.waypoints = []
        for p in raw:
            if isinstance(p, dict) and 'x' in p:
                self.waypoints.append((p['x'], p['y']))
            elif isinstance(p, dict) and 'position' in p:
                pos = p['position']
                if isinstance(pos, list):
                    self.waypoints.append((pos[0], pos[1]))
                else:
                    self.waypoints.append((pos['x'], pos['y']))
        # Downsample to ~20 representative waypoints to avoid micro-jitter
        step = max(1, len(self.waypoints) // 20)
        self.waypoints = self.waypoints[::step]
        # Always include the final point
        raw_last = raw[-1]
        if isinstance(raw_last, dict) and 'x' in raw_last:
            last = (raw_last['x'], raw_last['y'])
        else:
            pos = raw_last['position']
            last = (pos[0], pos[1]) if isinstance(pos, list) else (pos['x'], pos['y'])
        if self.waypoints[-1] != last:
            self.waypoints.append(last)

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {path_file}')

        self.pos = None
        self.yaw = None
        self.wp_idx = 0
        self.done = False
        self.actual_path = []

        self.cmd_pub = self.create_subscription   # placeholder
        self.cmd_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/rover/odom', self._odom_cb, _QOS)

        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info('RoverDriver ready — waiting for odometry…')

    # ────────────────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pos = (p.x, p.y)
        self.yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

    # ────────────────────────────────────────────────────────────────────────
    def _tick(self):
        if self.done or self.pos is None:
            return

        x, y = self.pos

        # Record path sample
        self.actual_path.append({'x': round(x, 4), 'y': round(y, 4)})

        if self.wp_idx >= len(self.waypoints):
            self._finish()
            return

        wx, wy = self.waypoints[self.wp_idx]
        dx, dy = wx - x, wy - y
        dist = math.hypot(dx, dy)

        if dist < WP_REACH:
            self.get_logger().info(
                f'Waypoint {self.wp_idx+1}/{len(self.waypoints)} reached '
                f'({wx:.2f}, {wy:.2f})')
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                self._finish()
            return

        desired_yaw = math.atan2(dy, dx)
        angle_err = _angle_diff(desired_yaw, self.yaw)

        # Report large turns — indicates path detouring around an obstacle
        if abs(angle_err) > 0.5 and not getattr(self, '_last_turn_reported', False):
            deg = math.degrees(angle_err)
            self.get_logger().info(
                f'Turning {deg:+.1f} deg to avoid obstacle cluster '
                f'(wp {self.wp_idx+1}/{len(self.waypoints)})')
            self._last_turn_reported = True
        elif abs(angle_err) <= 0.3:
            self._last_turn_reported = False

        twist = Twist()
        twist.angular.z = KP_YAW * angle_err
        if abs(angle_err) < ANGLE_OK:
            twist.linear.x = FORWARD_SPEED
        self.cmd_pub.publish(twist)

    # ────────────────────────────────────────────────────────────────────────
    def _finish(self):
        self.done = True
        self.cmd_pub.publish(Twist())   # stop
        out_file = os.path.join(RESULTS_DIR, 'rover_actual_path.json')
        with open(out_file, 'w') as f:
            json.dump(self.actual_path, f, indent=2)
        self.get_logger().info(
            f'Navigation complete — {len(self.actual_path)} path points '
            f'saved to {out_file}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RoverDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
