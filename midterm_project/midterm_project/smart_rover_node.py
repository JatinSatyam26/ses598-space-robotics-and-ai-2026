#!/usr/bin/env python3
"""
Smart Rover Node — Waits for drone's occupancy grid, plans A* path to habitat dome,
navigates with proportional control, uses LiDAR for reactive local obstacle avoidance.

State machine: WAITING → PLANNING → NAVIGATING → ARRIVED
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import heapq


class SmartRoverNode(Node):
    def __init__(self):
        super().__init__('smart_rover_node')

        gz_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.create_subscription(Odometry, '/rover/odom', self.odom_cb, gz_qos)
        self.create_subscription(LaserScan, '/rover/lidar', self.lidar_cb, gz_qos)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(OccupancyGrid, '/drone/occupancy_grid', self.map_cb, map_qos)

        self.pos = np.array([0.0, 1.5, 0.0])
        self.yaw = 0.0
        self.has_odom = False

        self.occupancy_grid = None
        self.grid_info = None
        self.path = []
        self.path_idx = 0
        self.state = 'WAITING'

        # LiDAR state
        self.lidar_msg = None
        self.lidar_min_front = float('inf')
        self.lidar_avoid_dir = 0.0   # +1 = turn left, -1 = turn right
        self.avoid_ticks = 0         # how many control cycles to keep avoiding

        # Mission parameters
        self.goal = np.array([15.0, 0.0])  # dome center, from world SDF
        self.wp_tolerance = 0.5
        self.arrival_tolerance = 1.8    # dome_radius(1.0) + rover_half_len(0.6) + margin(0.2)

        # Safety thresholds
        self.obstacle_stop_dist = 0.55    # stop and avoid if closer than this
        self.obstacle_caution_dist = 0.9  # slow down if closer than this

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('SmartRoverNode initialized — waiting for occupancy grid...')

    def odom_cb(self, msg):
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.has_odom = True

    def lidar_cb(self, msg):
        self.lidar_msg = msg
        n = len(msg.ranges)
        if n == 0:
            return

        # Build range array masking invalid readings
        ranges = np.array(msg.ranges)
        ranges[ranges < msg.range_min] = float('inf')
        ranges[ranges > msg.range_max] = float('inf')

        # Front sector: ±40° (covers indices 0-40 and 320-360 for 360-sample lidar)
        front_half = 40
        front_idx = list(range(0, front_half)) + list(range(n - front_half, n))
        front_vals = ranges[front_idx]
        self.lidar_min_front = float(np.nanmin(front_vals)) if len(front_vals) else float('inf')

        # Determine avoidance direction: compare left vs right clearance
        # Left sector: indices 40-140, Right sector: indices 220-320 (for 360-sample)
        left_min = float(np.nanmin(ranges[40:140])) if n >= 140 else float('inf')
        right_min = float(np.nanmin(ranges[220:320])) if n >= 320 else float('inf')
        # Turn toward the side with more clearance
        self.lidar_avoid_dir = 1.0 if left_min > right_min else -1.0

    def map_cb(self, msg):
        if self.occupancy_grid is not None:
            return  # only accept first map
        self.grid_info = msg.info
        w, h = msg.info.width, msg.info.height
        self.occupancy_grid = np.array(msg.data, dtype=np.int8).reshape(h, w)
        occupied = int(np.sum(self.occupancy_grid > 0))
        self.get_logger().info(
            f'Occupancy grid received: {w}x{h}, {occupied} occupied cells')
        self.state = 'PLANNING'

    def _world_to_grid(self, wx, wy):
        gx = int((wx - self.grid_info.origin.position.x) / self.grid_info.resolution)
        gy = int((wy - self.grid_info.origin.position.y) / self.grid_info.resolution)
        return gx, gy

    def _grid_to_world(self, gx, gy):
        wx = (gx * self.grid_info.resolution
              + self.grid_info.origin.position.x
              + self.grid_info.resolution / 2)
        wy = (gy * self.grid_info.resolution
              + self.grid_info.origin.position.y
              + self.grid_info.resolution / 2)
        return wx, wy

    def _astar(self, start_w, goal_w):
        sx, sy = self._world_to_grid(start_w[0], start_w[1])
        gx, gy = self._world_to_grid(goal_w[0], goal_w[1])
        h, w = self.occupancy_grid.shape

        # Inflate obstacles by 3 cells (0.75m margin) for rover safety
        inflated = np.copy(self.occupancy_grid)
        obstacle_mask = self.occupancy_grid > 20
        for dy in range(-3, 4):
            for dx in range(-3, 4):
                shifted = np.roll(np.roll(obstacle_mask, dy, axis=0), dx, axis=1)
                inflated[shifted] = 100

        sx = int(np.clip(sx, 0, w - 1))
        sy = int(np.clip(sy, 0, h - 1))
        gx = int(np.clip(gx, 0, w - 1))
        gy = int(np.clip(gy, 0, h - 1))

        # Find nearest free cell to goal if it's occupied
        if inflated[gy, gx] > 20:
            self.get_logger().warn('Goal in obstacle zone — searching for free cell')
            found = False
            for r in range(1, 25):
                for ddy in range(-r, r + 1):
                    for ddx in range(-r, r + 1):
                        ny, nx = gy + ddy, gx + ddx
                        if 0 <= ny < h and 0 <= nx < w and inflated[ny, nx] <= 20:
                            gx, gy = nx, ny
                            found = True
                            break
                    if found:
                        break
                if found:
                    break

        # A* search
        open_set = [(0.0, sx, sy)]
        came_from = {}
        g_score = {(sx, sy): 0.0}
        closed = set()

        while open_set:
            _, cx, cy = heapq.heappop(open_set)
            if (cx, cy) in closed:
                continue
            closed.add((cx, cy))

            if cx == gx and cy == gy:
                path = []
                cur = (gx, gy)
                while cur in came_from:
                    wx_pt, wy_pt = self._grid_to_world(cur[0], cur[1])
                    path.append((wx_pt, wy_pt))
                    cur = came_from[cur]
                path.reverse()
                if len(path) > 8:
                    sparse = path[::4]
                    if sparse[-1] != path[-1]:
                        sparse.append(path[-1])
                    return sparse
                return path

            for ddx, ddy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                              (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nx, ny = cx + ddx, cy + ddy
                if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in closed:
                    cell_val = int(inflated[ny, nx])
                    if cell_val > 20:
                        continue
                    # Unknown cells (-1) are passable but cost 3× to prefer surveyed corridors
                    unknown_mult = 3.0 if cell_val < 0 else 1.0
                    cost = (1.414 if (ddx != 0 and ddy != 0) else 1.0) * unknown_mult
                    ng = g_score[(cx, cy)] + cost
                    if ng < g_score.get((nx, ny), float('inf')):
                        g_score[(nx, ny)] = ng
                        f = ng + math.sqrt((nx - gx)**2 + (ny - gy)**2)
                        heapq.heappush(open_set, (f, nx, ny))
                        came_from[(nx, ny)] = (cx, cy)

        self.get_logger().error('A* found no path!')
        return []

    def control_loop(self):
        cmd = Twist()

        if self.state == 'WAITING':
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'PLANNING':
            if not self.has_odom:
                return
            self.path = self._astar(self.pos[:2], self.goal)
            if not self.path:
                self.get_logger().error('No path found — staying in WAITING')
                self.state = 'WAITING'
                return
            self.path_idx = 0
            self.get_logger().info(f'Path planned: {len(self.path)} waypoints')
            for i, (wx, wy) in enumerate(self.path):
                self.get_logger().info(f'  WP {i:2d}: ({wx:.1f}, {wy:.1f})')
            self.state = 'NAVIGATING'
            return

        if self.state == 'NAVIGATING':
            # Path-completion arrival (Approach B). The rover arrives when it
            # has advanced through every planned waypoint. Straight-line
            # distance to goal is NOT used — it produces false positives on
            # curved approaches. The final waypoint is also sanity-checked
            # against the actual dome position so we never declare success
            # when the planner routed us to the wrong place.
            if self.path and self.path_idx >= len(self.path):
                final_wp = self.path[-1]
                dist_final_to_dome = math.sqrt(
                    (final_wp[0] - self.goal[0])**2 +
                    (final_wp[1] - self.goal[1])**2)

                MAX_FINAL_WP_DIST_FROM_DOME = 2.0  # dome_radius + grid cell + margin
                if dist_final_to_dome <= MAX_FINAL_WP_DIST_FROM_DOME:
                    self.state = 'ARRIVED'
                    self.get_logger().info(
                        f'ARRIVED at habitat dome! Mission complete! '
                        f'(final WP {final_wp} is {dist_final_to_dome:.2f}m '
                        f'from dome center)')
                    self.cmd_pub.publish(Twist())
                    return
                else:
                    self.get_logger().error(
                        f'Path completed but final WP {final_wp} is '
                        f'{dist_final_to_dome:.2f}m from dome center — '
                        f'A* did not route to the dome. NOT declaring arrival.')
                    self.cmd_pub.publish(Twist())
                    return

            # ── Reactive LiDAR obstacle avoidance ──────────────────────────────
            if self.lidar_min_front < self.obstacle_stop_dist:
                # Actively avoid: turn toward the clearer side, creep back slightly
                self.avoid_ticks = 15  # persist avoidance for 1.5s
                self.get_logger().warn(
                    f'Obstacle at {self.lidar_min_front:.2f}m — avoiding '
                    f'(turn {"left" if self.lidar_avoid_dir > 0 else "right"})')

            if self.avoid_ticks > 0:
                self.avoid_ticks -= 1
                cmd.linear.x = 0.0
                cmd.angular.z = float(np.clip(self.lidar_avoid_dir * 1.2, -1.5, 1.5))
                # Brief reverse if very close
                if self.lidar_min_front < 0.4:
                    cmd.linear.x = -0.1
                self.cmd_pub.publish(cmd)
                return
            # ──────────────────────────────────────────────────────────────────

            # Navigate to current waypoint
            if self.path_idx >= len(self.path):
                tx, ty = self.goal
            else:
                tx, ty = self.path[self.path_idx]

            dx = tx - self.pos[0]
            dy = ty - self.pos[1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < self.wp_tolerance and self.path_idx < len(self.path):
                self.path_idx += 1
                self.get_logger().info(
                    f'Waypoint {self.path_idx}/{len(self.path)} reached')
                return

            desired_yaw = math.atan2(dy, dx)
            yaw_error = desired_yaw - self.yaw
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            # Slow down if obstacle nearby in caution zone
            caution = max(0.3, min(1.0, (self.lidar_min_front - self.obstacle_stop_dist)
                                   / (self.obstacle_caution_dist - self.obstacle_stop_dist)))

            if abs(yaw_error) > 0.5:
                cmd.angular.z = float(np.clip(yaw_error * 2.0, -1.5, 1.5))
                cmd.linear.x = 0.05
            else:
                max_speed = 0.4 * caution
                cmd.linear.x = float(min(max_speed, dist * 0.5))
                cmd.angular.z = float(np.clip(yaw_error * 1.5, -1.0, 1.0))

            self.cmd_pub.publish(cmd)
            return

        if self.state == 'ARRIVED':
            self.cmd_pub.publish(Twist())
            return

    def destroy_node(self):
        self.cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SmartRoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
