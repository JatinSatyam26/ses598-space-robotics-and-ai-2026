#!/usr/bin/env python3
"""
rover_navigator.py — Nav2-driven rover mission: spawn point → goal across rock field.

Uses BasicNavigator (nav2_simple_commander) to:
  1. Wait for Nav2 to be active (skips AMCL — we use Gazebo odometry directly)
  2. Set initial pose at rover spawn (8, 0)
  3. Send goal to (-5, -5)
  4. Monitor progress every second: distance remaining, current pose, ETA
  5. Record the traversed path (sampled from feedback) to ~/midterm_results/rover_path.json
  6. On completion: summary with total distance, elapsed time, success/failure

Run with:
  ros2 run midterm_project rover_navigator
or:
  ros2 run midterm_project rover_navigator --ros-args \
      -p goal_x:=-8.0 -p goal_y:=5.0
"""

import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# ── Configurable defaults ──────────────────────────────────────────────────────

START_X =  8.0    # rover spawn (matches model.sdf and rover_nav.launch.py)
START_Y =  0.0
GOAL_X  = -5.0    # other side of the rock field
GOAL_Y  = -5.0

RESULTS_DIR = Path.home() / 'midterm_results'
MONITOR_HZ  = 1.0   # progress prints per second
PATH_SAMPLE_DIST_M = 0.3   # record a waypoint every ~0.3 m of movement


# ── Helpers ────────────────────────────────────────────────────────────────────

def make_pose(x: float, y: float, yaw: float = 0.0,
              frame: str = 'map') -> PoseStamped:
    """Build a stamped pose with a yaw-only quaternion."""
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = 0.0
    # quaternion from yaw: q = (0, 0, sin(yaw/2), cos(yaw/2))
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def _pose_to_dict(pose) -> dict:
    """Convert a geometry_msgs/Pose or PoseStamped .pose to a JSON-serialisable dict."""
    if hasattr(pose, 'pose'):
        pose = pose.pose
    return {
        'x': round(pose.position.x, 4),
        'y': round(pose.position.y, 4),
        'z': round(pose.position.z, 4),
        'qx': round(pose.orientation.x, 6),
        'qy': round(pose.orientation.y, 6),
        'qz': round(pose.orientation.z, 6),
        'qw': round(pose.orientation.w, 6),
    }


def _dist2d(a, b) -> float:
    """Euclidean XY distance between two dicts with 'x' and 'y' keys."""
    return math.hypot(a['x'] - b['x'], a['y'] - b['y'])


def _duration_to_sec(duration_msg) -> float:
    """Convert builtin_interfaces/Duration to seconds."""
    return duration_msg.sec + duration_msg.nanosec * 1e-9


# ── Navigator node ─────────────────────────────────────────────────────────────

class RoverNavigator(Node):
    """
    Thin wrapper around BasicNavigator that adds:
      - ROS2 parameter declarations for start/goal
      - Progress monitoring with path recording
      - JSON result export
    """

    def __init__(self):
        super().__init__('rover_navigator')

        # Parameters (overridable via --ros-args -p goal_x:=X)
        self.declare_parameter('start_x', START_X)
        self.declare_parameter('start_y', START_Y)
        self.declare_parameter('goal_x',  GOAL_X)
        self.declare_parameter('goal_y',  GOAL_Y)
        self.declare_parameter('results_dir', str(RESULTS_DIR))
        self.declare_parameter('nav_timeout_sec', 300.0)   # 5-min hard limit

        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x  = self.get_parameter('goal_x').value
        self.goal_y  = self.get_parameter('goal_y').value
        self.results_dir = Path(self.get_parameter('results_dir').value)
        self.nav_timeout_sec = self.get_parameter('nav_timeout_sec').value

        self.results_dir.mkdir(parents=True, exist_ok=True)

    def run_mission(self):
        """Execute the full A→B navigation mission. Returns True on success."""

        # ── 1. Initialise BasicNavigator ──────────────────────────────────────
        nav = BasicNavigator()
        nav.get_logger().info(
            f'RoverNavigator: ({self.start_x}, {self.start_y}) → '
            f'({self.goal_x}, {self.goal_y})')

        # ── 2. Set initial pose ───────────────────────────────────────────────
        # Publishes to /initialpose. Without AMCL this just seeds the navigator's
        # internal state; real localisation comes from /rover/odom → /odom → TF.
        initial_pose = make_pose(self.start_x, self.start_y, yaw=0.0)
        initial_pose.header.stamp = nav.get_clock().now().to_msg()
        nav.setInitialPose(initial_pose)

        # ── 3. Wait for Nav2 ──────────────────────────────────────────────────
        # Pass localizer='robot_localization' to bypass the AMCL lifecycle wait
        # (robot_localization is treated as a non-lifecycle node so the check
        # is skipped — see BasicNavigator.waitUntilNav2Active() source).
        nav.get_logger().info('Waiting for Nav2 to become active …')
        nav.waitUntilNav2Active(
            navigator='bt_navigator',
            localizer='robot_localization'
        )
        nav.get_logger().info('Nav2 is active.')

        # ── 4. Send goal ──────────────────────────────────────────────────────
        goal_pose = make_pose(self.goal_x, self.goal_y, yaw=0.0)
        goal_pose.header.stamp = nav.get_clock().now().to_msg()

        nav.get_logger().info(
            f'Sending goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')
        accepted = nav.goToPose(goal_pose)
        if not accepted:
            nav.get_logger().error(
                'Goal was REJECTED by Nav2. '
                'Check that the goal cell is not marked as obstacle/unknown '
                'in the costmap, and that a path exists.')
            nav.destroy_node()
            return False

        # ── 5. Monitor progress ───────────────────────────────────────────────
        t_start          = time.time()
        t_last_print     = t_start
        t_last_sample    = t_start
        recorded_path    = []   # list of {x,y,z,...,timestamp}
        last_sampled_pos = {'x': self.start_x, 'y': self.start_y}
        total_dist_traveled = 0.0
        recoveries_seen  = 0
        last_dist_remaining = None

        nav.get_logger().info('Mission running — monitoring progress …')

        while not nav.isTaskComplete():
            now = time.time()
            elapsed = now - t_start

            # Hard timeout
            if elapsed > self.nav_timeout_sec:
                nav.get_logger().error(
                    f'TIMEOUT: mission exceeded {self.nav_timeout_sec:.0f}s. '
                    f'Cancelling goal.')
                nav.cancelTask()
                break

            feedback = nav.getFeedback()
            if feedback is None:
                time.sleep(1.0 / MONITOR_HZ)
                continue

            fb    = feedback.feedback            # NavigateToPose_FeedbackMessage
            cur   = fb.current_pose              # PoseStamped
            dist  = float(fb.distance_remaining) # metres
            recs  = int(fb.number_of_recoveries)
            nav_t = _duration_to_sec(fb.navigation_time)
            eta_s = _duration_to_sec(fb.estimated_time_remaining)

            cur_dict = _pose_to_dict(cur)
            last_dist_remaining = dist

            # ── Record path sample ───────────────────────────────────────────
            if _dist2d(cur_dict, last_sampled_pos) >= PATH_SAMPLE_DIST_M:
                sample = dict(cur_dict)
                sample['timestamp'] = round(elapsed, 2)
                sample['dist_remaining'] = round(dist, 3)
                recorded_path.append(sample)
                total_dist_traveled += _dist2d(cur_dict, last_sampled_pos)
                last_sampled_pos = cur_dict

            # ── Console progress ─────────────────────────────────────────────
            if now - t_last_print >= 1.0 / MONITOR_HZ:
                eta_str = f'{eta_s:.0f}s' if eta_s >= 0 else 'N/A'
                recovery_note = (
                    f'  ⚠ RECOVERY #{recs}' if recs > recoveries_seen else '')
                recoveries_seen = max(recoveries_seen, recs)

                print(
                    f'  [{elapsed:6.1f}s] '
                    f'pos=({cur_dict["x"]:+7.3f}, {cur_dict["y"]:+7.3f})  '
                    f'dist_remaining={dist:6.2f}m  '
                    f'ETA={eta_str}  '
                    f'recoveries={recs}'
                    f'{recovery_note}')
                t_last_print = now

            time.sleep(0.1)   # 10 Hz poll — isTaskComplete has 0.1 s internal timeout

        # ── 6. Final result ───────────────────────────────────────────────────
        elapsed_total = time.time() - t_start
        result = nav.getResult()

        # Append final position to path
        feedback = nav.getFeedback()
        if feedback is not None:
            final_pos = _pose_to_dict(feedback.feedback.current_pose)
            final_pos['timestamp'] = round(elapsed_total, 2)
            final_pos['dist_remaining'] = 0.0
            recorded_path.append(final_pos)

        success = (result == TaskResult.SUCCEEDED)

        print()
        print('=' * 60)
        if success:
            print('  MISSION COMPLETE — SUCCESS')
        elif result == TaskResult.CANCELED:
            print('  MISSION ENDED — CANCELLED (timeout or manual cancel)')
        else:
            # FAILED: dig out why
            print('  MISSION FAILED')
            if last_dist_remaining is not None:
                print(f'  Stuck at ~{last_dist_remaining:.2f}m from goal')
            if recoveries_seen > 0:
                print(f'  Attempted {recoveries_seen} recovery behaviour(s) — '
                      f'all exhausted')
            print('  Likely causes: goal/path in obstacle/unknown zone, '
                  'or DWB could not find a valid trajectory (robot too close '
                  'to obstacle, costmap inflation too aggressive, or the '
                  'planned path went through an untraversable cell).')
            print('  Fix suggestions:')
            print('    1. Check costmap_debug.png — is goal cell green (free)?')
            print('    2. Increase --inflation-radius in generate_costmap.py if '
                  '   path is clipped')
            print('    3. Try an intermediate waypoint that avoids the rock field')

        print(f'  Result          : {result.name}')
        print(f'  Time taken      : {elapsed_total:.1f}s')
        print(f'  Distance traveled (approx): {total_dist_traveled:.2f}m')
        print(f'  Recovery attempts: {recoveries_seen}')
        print(f'  Path samples    : {len(recorded_path)}')
        print('=' * 60)

        # ── 7. Save results ───────────────────────────────────────────────────
        mission_data = {
            'success': success,
            'result': result.name,
            'start': {'x': self.start_x, 'y': self.start_y},
            'goal':  {'x': self.goal_x,  'y': self.goal_y},
            'elapsed_sec': round(elapsed_total, 2),
            'total_dist_traveled_m': round(total_dist_traveled, 3),
            'recoveries': recoveries_seen,
            'path': recorded_path,
        }
        path_file = self.results_dir / 'rover_path.json'
        with open(path_file, 'w') as f:
            json.dump(mission_data, f, indent=2)
        print(f'  Path saved to   : {path_file}')

        nav.destroy_node()
        return success


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RoverNavigator()
    try:
        success = node.run_mission()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
        success = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
