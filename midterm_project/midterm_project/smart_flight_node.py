#!/usr/bin/env python3
"""
Smart Flight Node — Drone surveys terrain at 2m AGL, builds occupancy grid
from rangefinder height map, publishes OccupancyGrid for rover, then returns
home and lands.

State machine: TAKEOFF → SURVEY → RETURN_HOME → LAND → DONE
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Header
import numpy as np
import math
import time
import subprocess


class SmartFlightNode(Node):
    def __init__(self):
        super().__init__('smart_flight_node')

        gz_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/drone/occupancy_grid', 10)

        self.create_subscription(Odometry, '/drone/odom', self.odom_cb, gz_qos)
        self.create_subscription(LaserScan, '/drone/rangefinder', self.range_cb, gz_qos)
        self.create_subscription(Image, '/drone/front_depth', self.depth_cb, gz_qos)

        # State
        self.state = 'TAKEOFF'
        self.pos = np.array([0.0, 0.0, 0.0])
        self.yaw = 0.0
        self.rangefinder_alt = 0.0
        self.has_odom = False
        self.has_range = False
        self.odom_origin = None

        # Survey at 3m AGL — rocks are 0.35-0.55m tall, giving 2.45-2.65m rangefinder
        # readings. Detection threshold (survey_alt - 0.25 = 2.75m) catches anything > 0.25m tall.
        self.survey_alt = 3.0
        self.land_home = np.array([1.0, 0.0])  # XY to fly back to before landing

        # Occupancy grid
        self.grid_resolution = 0.25
        self.grid_x_min, self.grid_x_max = -2.0, 18.0
        self.grid_y_min, self.grid_y_max = -5.0, 5.0
        self.grid_width = int((self.grid_x_max - self.grid_x_min) / self.grid_resolution)
        self.grid_height = int((self.grid_y_max - self.grid_y_min) / self.grid_resolution)
        self.occupancy_counts = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)

        # Survey lanes — 5 lanes at y=-2, -1, 0, 1, 2 for denser coverage
        self.survey_waypoints = self._generate_lawnmower()
        self.wp_idx = 0
        self.wp_tolerance = 0.5

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f'SmartFlightNode initialized — survey_alt={self.survey_alt}m, '
            f'grid={self.grid_width}x{self.grid_height}')

    def _generate_lawnmower(self):
        waypoints = []
        y_vals = [-2.0, -1.0, 0.0, 1.0, 2.0]  # 5 lanes for denser coverage
        for i, y in enumerate(y_vals):
            if i % 2 == 0:
                waypoints.append((2.0, y, self.survey_alt))
                waypoints.append((14.0, y, self.survey_alt))
            else:
                waypoints.append((14.0, y, self.survey_alt))
                waypoints.append((2.0, y, self.survey_alt))
        return waypoints

    def odom_cb(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        if self.odom_origin is None:
            self.odom_origin = np.array([px, py])
        self.pos[0] = px - self.odom_origin[0]
        self.pos[1] = py - self.odom_origin[1]
        self.pos[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.has_odom = True

    def range_cb(self, msg):
        if msg.ranges and len(msg.ranges) > 0 and msg.ranges[0] > msg.range_min:
            self.rangefinder_alt = msg.ranges[0]
            self.has_range = True

    def depth_cb(self, msg):
        if not self.has_odom or self.state != 'SURVEY':
            return
        try:
            if msg.encoding == '32FC1':
                depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                    msg.height, msg.width)
            elif msg.encoding == '16UC1':
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    msg.height, msg.width).astype(np.float32) / 1000.0
            else:
                return
            center_row = msg.height // 2
            strip = depth[center_row - 5:center_row + 5, :]
            fx = msg.width / (2.0 * math.tan(1.047 / 2.0))
            for col in range(0, msg.width, 4):
                d = np.nanmedian(strip[:, col])
                if np.isnan(d) or d < 0.3 or d > 10.0:
                    continue
                angle = math.atan2(col - msg.width / 2.0, fx) + self.yaw
                wx = self.pos[0] + d * math.cos(angle)
                wy = self.pos[1] + d * math.sin(angle)
                if d < self.pos[2] - 0.3:
                    gx = int((wx - self.grid_x_min) / self.grid_resolution)
                    gy = int((wy - self.grid_y_min) / self.grid_resolution)
                    if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                        self.occupancy_counts[gy, gx] += 1
        except Exception as e:
            self.get_logger().warn(f'Depth error: {e}')

    def _mark_obstacle(self, gx, gy, weight=2, radius=2):
        """Mark obstacle at (gx,gy) and surrounding cells in the grid."""
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                    self.occupancy_counts[ny, nx] += weight

    def control_loop(self):
        cmd = Twist()

        if self.state == 'TAKEOFF':
            if not self.has_odom:
                return
            cmd.linear.z = 1.5
            if self.pos[2] >= self.survey_alt - 0.2:
                self.state = 'SURVEY'
                self.get_logger().info(
                    f'TAKEOFF complete z={self.pos[2]:.2f}m — starting SURVEY '
                    f'({len(self.survey_waypoints)} waypoints)')
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'SURVEY':
            if self.wp_idx >= len(self.survey_waypoints):
                self.state = 'RETURN_HOME'
                self.get_logger().info('SURVEY complete — publishing map, returning home')
                self._publish_map()
                return

            tx, ty, _ = self.survey_waypoints[self.wp_idx]
            dx = tx - self.pos[0]
            dy = ty - self.pos[1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < self.wp_tolerance:
                self.wp_idx += 1
                self.get_logger().info(
                    f'Waypoint {self.wp_idx}/{len(self.survey_waypoints)} reached')
                return

            speed = min(2.0, dist)
            cmd.linear.x = speed * dx / dist
            cmd.linear.y = speed * dy / dist

            # Altitude hold
            if self.has_range:
                alt_error = self.survey_alt - self.rangefinder_alt
                cmd.linear.z = float(np.clip(alt_error * 1.2, -0.8, 0.8))

                # -- Primary obstacle detection via rangefinder --
                # At 2m survey alt, rocks (0.3-0.5m tall) give readings of 1.5-1.7m.
                # Threshold 0.25m below survey alt catches anything > 0.25m tall.
                if self.rangefinder_alt < (self.survey_alt - 0.25):
                    gx = int((self.pos[0] - self.grid_x_min) / self.grid_resolution)
                    gy = int((self.pos[1] - self.grid_y_min) / self.grid_resolution)
                    self._mark_obstacle(gx, gy, weight=4, radius=2)
                    self.get_logger().debug(
                        f'Obstacle below! range={self.rangefinder_alt:.2f}m '
                        f'at ({self.pos[0]:.1f},{self.pos[1]:.1f})')

                if self.rangefinder_alt < 1.5:
                    cmd.linear.z = 1.5  # emergency climb
                    self.get_logger().warn(
                        f'CRITICAL CLEARANCE {self.rangefinder_alt:.2f}m — climbing!')
            else:
                alt_error = self.survey_alt - self.pos[2]
                cmd.linear.z = float(np.clip(alt_error * 1.2, -0.8, 0.8))

            self.cmd_pub.publish(cmd)
            return

        if self.state == 'RETURN_HOME':
            dx = self.land_home[0] - self.pos[0]
            dy = self.land_home[1] - self.pos[1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < 0.6:
                self.state = 'LAND'
                self.get_logger().info(
                    f'At home position ({self.pos[0]:.1f},{self.pos[1]:.1f}) — LANDING')
                return

            speed = min(2.0, dist)
            cmd.linear.x = speed * dx / dist
            cmd.linear.y = speed * dy / dist

            # Maintain altitude during return
            if self.has_range:
                alt_error = self.survey_alt - self.rangefinder_alt
                cmd.linear.z = float(np.clip(alt_error * 1.2, -0.8, 0.8))
            else:
                cmd.linear.z = float(np.clip((self.survey_alt - self.pos[2]) * 1.2,
                                             -0.8, 0.8))
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'LAND':
            # Aggressive descent — hold near zero XY, push down firmly
            cmd.linear.x = -self.pos[0] * 0.5  # drift correction
            cmd.linear.y = -self.pos[1] * 0.5
            cmd.linear.z = -1.0

            if self.pos[2] < 0.23:
                self.state = 'DONE'
                self.get_logger().info(
                    f'LANDED at z={self.pos[2]:.3f}m — disabling controller')

                # Stop the velocity controller plugin. MulticopterVelocityControl
                # has no disarm primitive; publishing zero Twist only commands zero
                # velocity, which the controller holds imperfectly (observed ~4 mm/s
                # drift in Apr 16 run). Disabling the plugin outright halts all
                # control authority — rotors spin down, drone settles.
                try:
                    result = subprocess.run(
                        ['gz', 'topic', '-t', '/flying_drone/enable',
                         '--msgtype', 'gz.msgs.Boolean',
                         '-p', 'data: false'],
                        capture_output=True, timeout=2.0, text=True
                    )
                    if result.returncode == 0:
                        self.get_logger().info('Controller disabled — drone safe')
                    else:
                        self.get_logger().error(
                            f'Disable failed (rc={result.returncode}): {result.stderr}')
                except Exception as e:
                    self.get_logger().error(f'Disable exception: {e}')

                # Publish zero Twist once for completeness
                self.cmd_pub.publish(Twist())
                return
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'DONE':
            # Controller is disabled. No work needed. Node stays alive so the
            # rover's ROS2 graph remains healthy.
            return

    def _publish_map(self):
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'world'
        grid.info.resolution = self.grid_resolution
        grid.info.width = self.grid_width
        grid.info.height = self.grid_height
        grid.info.origin.position.x = self.grid_x_min
        grid.info.origin.position.y = self.grid_y_min
        grid.info.origin.position.z = 0.0

        flat = self.occupancy_counts.flatten()
        data = np.zeros(len(flat), dtype=np.int8)
        max_count = max(flat.max(), 1)
        for i in range(len(flat)):
            if flat[i] > 0:
                data[i] = min(100, int(flat[i] * 100 / max_count))

        grid.data = data.tolist()
        self.map_pub.publish(grid)

        occupied = int(np.sum(flat > 0))
        self.get_logger().info(
            f'OccupancyGrid published: {self.grid_width}x{self.grid_height}, '
            f'{occupied} occupied cells')

        # Re-publish several times for reliability
        for i in range(5):
            time.sleep(0.4)
            grid.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = SmartFlightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
