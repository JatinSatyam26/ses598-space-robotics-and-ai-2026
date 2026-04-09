#!/usr/bin/env python3
"""
control_node.py — SES 598 Midterm v5

ROOT CAUSES FROM ALL LOGS:
1. VehicleStatus pre_flight_checks_pass NEVER becomes True:
   - Either field doesn't exist in this PX4 version's px4_msgs
   - OR EKF accel bias never clears due to time sync jumps
2. Time sync jumps caused by /clock bridge injecting Gazebo time into ROS2

FIXES v5:
- Drop VehicleStatus entirely — not reliable for this PX4 version
- Remove clock bridge (done in run_term3.sh) — stops time sync interference
- Progressive arm strategy: try normal arm, then force arm (param2=21196.0)
- Detect arm success via altitude rising — reliable regardless of VehicleStatus
- Timeout watchdog: if stuck in any state too long → log and exit cleanly

PHASE TIMEOUTS (derived from A3 trial data):
  PREFLIGHT first arm attempt : 30s    (after 30s of heartbeat streaming)
  PREFLIGHT total             : 300s   (5 min — force arm after this)
  WAIT_HOVER                  : 30s    (Auto.Takeoff should complete in <15s)
  SWITCH_OFFBOARD             : 60s    (Offboard mode accept)
  TAKEOFF to altitude         : 30s    (climb 8m at normal rate)
  Per waypoint                : 90s    (each of 14 waypoints)
  Total mission               : 1200s  (20 min hard limit)
"""

import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, TrajectorySetpoint

SURVEY_ALTITUDE_M    = 5.0
POSITION_THRESHOLD_M = 2.0
ALTITUDE_THRESHOLD_M = 0.5
HOVER_DURATION_S     = 10.0
WP_HOVER_DURATION_S  = 3.0  # hover at each survey waypoint before advancing

# Timeouts (seconds)
T_FIRST_ARM_ATTEMPT  = 30    # send first arm attempt after 30s of streaming
T_FORCE_ARM          = 120   # switch to force arm after 2 min of normal arm failures
T_WAIT_HOVER         = 30    # timeout for Auto.Takeoff to reach 4.5m
T_SWITCH_OFFBOARD    = 60    # timeout for Offboard mode acceptance
T_TAKEOFF            = 30    # timeout for climbing to 8m
T_WAYPOINT           = 90    # timeout per waypoint
T_MISSION_TOTAL      = 1200  # 20 min hard stop

# Boustrophedon survey: ±10m extent, 3m row spacing
_SURVEY_OFFSETS = [
    (-10.0,  0.0), (10.0,  0.0),
    ( 10.0,  3.0), (-10.0,  3.0),
    (-10.0,  6.0), (10.0,  6.0),
    ( 10.0,  9.0), (-10.0,  9.0),
    (-10.0, 12.0), (10.0, 12.0),
]

class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.trajectory_pub    = self.create_publisher(TrajectorySetpoint,   "/fmu/in/trajectory_setpoint",   qos)
        self.vehicle_cmd_pub   = self.create_publisher(VehicleCommand,       "/fmu/in/vehicle_command",       qos)
        self.odom_sub = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.odom_callback, qos)

        self.offboard_setpoint_counter = 0
        self.state            = "WAIT_ODOM"
        self.position         = None
        self.home_position    = None
        self.waypoints        = []
        self.waypoint_index   = 0
        self.hover_start_time = None
        self.wp_hover_start   = None  # per-waypoint 3s hover timer

        # Timing
        self.mission_start_time    = time.time()
        self.state_entry_time      = time.time()
        self.arm_attempts          = 0
        self.last_arm_time         = 0.0
        self.liftoff_detected      = False
        self.waypoint_start_time   = time.time()

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Control node initialized. Waiting for odometry...")

    def odom_callback(self, msg):
        try:
            self.position = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
            if self.home_position is None:
                self.home_position = list(self.position)
                z = self.home_position[2] - SURVEY_ALTITUDE_M
                self.waypoints = [(self.home_position[0]+dx, self.home_position[1]+dy, z) for dx,dy in _SURVEY_OFFSETS]
                self.get_logger().info(f"Home: x={self.home_position[0]:.2f} y={self.home_position[1]:.2f} z={self.home_position[2]:.2f}")
                self.get_logger().info(f"Survey: {len(self.waypoints)} waypoints at z={z:.2f} NED ({SURVEY_ALTITUDE_M}m AGL)")
        except (IndexError, AttributeError, TypeError):
            pass

    def _alt(self):
        if self.position is None or self.home_position is None:
            return 0.0
        return -(self.position[2] - self.home_position[2])

    def _dist3d(self, tx, ty, tz):
        if self.position is None:
            return float("inf")
        return math.sqrt((self.position[0]-tx)**2+(self.position[1]-ty)**2+(self.position[2]-tz)**2)

    def _state_elapsed(self):
        return time.time() - self.state_entry_time

    def _mission_elapsed(self):
        return time.time() - self.mission_start_time

    def _transition(self, new_state):
        self.get_logger().info(f"STATE: {self.state} → {new_state} (elapsed {self._state_elapsed():.1f}s)")
        self.state = new_state
        self.state_entry_time = time.time()

    def _timeout_exit(self, reason):
        elapsed = self._state_elapsed()
        total = self._mission_elapsed()
        self.get_logger().error(
            f"TIMEOUT in state={self.state} after {elapsed:.1f}s: {reason} "
            f"(total mission time: {total:.1f}s)")
        self.get_logger().error("MISSION ABORTED — check logs and restart")
        raise SystemExit(1)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position=True; msg.velocity=False; msg.acceleration=False; msg.attitude=False; msg.body_rate=False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0, velocity=None):
        msg = TrajectorySetpoint()
        msg.position=[float(x),float(y),float(z)]
        if velocity is not None:
            msg.velocity=[float(v) for v in velocity]
        else:
            msg.velocity=[float("nan"),float("nan"),float("nan")]
        msg.acceleration=[float("nan"),float("nan"),float("nan")]
        msg.yaw=float(yaw)
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command=command; msg.param1=float(param1); msg.param2=float(param2)
        msg.target_system=1; msg.target_component=1; msg.source_system=1; msg.source_component=1
        msg.from_external=True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_cmd_pub.publish(msg)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def arm_normal(self):
        """Standard arm — requires preflight checks to pass."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0, param2=0.0)
        self.arm_attempts += 1
        self.last_arm_time = time.time()
        self.get_logger().info(f"Arm attempt #{self.arm_attempts} (normal)")

    def arm_force(self):
        """Force arm — bypasses safety checks. Used after normal arm keeps failing."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0, param2=21196.0)
        self.arm_attempts += 1
        self.last_arm_time = time.time()
        self.get_logger().warn(f"Force arm attempt #{self.arm_attempts} (param2=21196.0)")

    def timer_callback(self):
        # Global mission timeout
        if self._mission_elapsed() > T_MISSION_TOTAL:
            self._timeout_exit(f"Total mission exceeded {T_MISSION_TOTAL}s")

        # ALWAYS: heartbeat + setpoint + offboard mode from tick 5
        self.publish_offboard_control_mode()
        if self.home_position is not None:
            self.publish_trajectory_setpoint(
                self.home_position[0], self.home_position[1],
                self.home_position[2] - SURVEY_ALTITUDE_M)
        else:
            self.publish_trajectory_setpoint(0.0, 0.0, -SURVEY_ALTITUDE_M)
        if self.offboard_setpoint_counter >= 5:
            self.engage_offboard_mode()

        # ── STATE MACHINE ────────────────────────────────────────────────────
        if self.state == "WAIT_ODOM":
            if self.position is not None and self.home_position is not None:
                self._transition("PREFLIGHT")
                self.get_logger().info(
                    f"Odometry received. PREFLIGHT — "
                    f"first arm in {T_FIRST_ARM_ATTEMPT}s, force arm at {T_FORCE_ARM}s...")

        elif self.state == "PREFLIGHT":
            elapsed = self._state_elapsed()
            alt = self._alt()

            # Progress log every 10s
            if self.offboard_setpoint_counter % 100 == 0:
                self.get_logger().info(
                    f"[PREFLIGHT] {elapsed:.0f}s elapsed | "
                    f"arm_attempts={self.arm_attempts} | alt={alt:.2f}m")

            # Try arm every 10s starting at T_FIRST_ARM_ATTEMPT
            arm_interval = 10.0
            time_since_last_arm = time.time() - self.last_arm_time
            should_try_arm = (elapsed >= T_FIRST_ARM_ATTEMPT and
                              time_since_last_arm >= arm_interval)

            if should_try_arm:
                if elapsed < T_FORCE_ARM:
                    self.arm_normal()
                else:
                    # Normal arm keeps failing — switch to force arm
                    self.arm_force()

            # Detect successful arm via liftoff (altitude rising)
            if alt > 0.3 and not self.liftoff_detected:
                self.liftoff_detected = True
                self.get_logger().info(f"✅ Liftoff detected at alt={alt:.2f}m after {elapsed:.1f}s!")
                self._transition("WAIT_HOVER")

            # Timeout: if armed many times but no liftoff for too long
            if self.arm_attempts > 10 and elapsed > T_FORCE_ARM + 60:
                self._timeout_exit(
                    f"Arm sent {self.arm_attempts} times over {elapsed:.0f}s "
                    f"but no liftoff detected. PX4 may need restart.")

        elif self.state == "WAIT_HOVER":
            alt = self._alt()
            elapsed = self._state_elapsed()
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"[WAIT_HOVER] alt={alt:.2f}m / 4.5m target")
            if alt >= 4.5:
                self._transition("SWITCH_OFFBOARD")
                self.get_logger().info(f"Auto.Takeoff complete at {alt:.2f}m. Switching to Offboard...")
            elif elapsed > T_WAIT_HOVER:
                self._timeout_exit(
                    f"Auto.Takeoff should reach 4.5m in {T_WAIT_HOVER}s but only got to {alt:.2f}m")

        elif self.state == "SWITCH_OFFBOARD":
            self.engage_offboard_mode()
            alt = self._alt()
            elapsed = self._state_elapsed()
            if self.offboard_setpoint_counter % 5 == 0:
                self.get_logger().info(f"[SWITCH_OFFBOARD] alt={alt:.2f}m / need >5.3m")
            if alt > 5.3:
                self._transition("TAKEOFF")
                self.get_logger().info(f"✅ Offboard mode accepted at {alt:.2f}m! Climbing to {SURVEY_ALTITUDE_M}m...")
            elif elapsed > T_SWITCH_OFFBOARD:
                self._timeout_exit(
                    f"Offboard mode not accepted after {T_SWITCH_OFFBOARD}s. Alt={alt:.2f}m")

        elif self.state == "TAKEOFF":
            tz = self.home_position[2] - SURVEY_ALTITUDE_M
            self.publish_trajectory_setpoint(self.home_position[0], self.home_position[1], tz)
            alt = self._alt()
            elapsed = self._state_elapsed()
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"[TAKEOFF] {alt:.2f}m / {SURVEY_ALTITUDE_M}m")
            if abs(alt - SURVEY_ALTITUDE_M) < ALTITUDE_THRESHOLD_M:
                self._transition("SURVEY")
                self.waypoint_index = 0
                self.waypoint_start_time = time.time()
                self.get_logger().info(
                    f"✅ Takeoff complete at {alt:.2f}m. Starting survey ({len(self.waypoints)} waypoints).")
            elif elapsed > T_TAKEOFF:
                self._timeout_exit(
                    f"Failed to reach {SURVEY_ALTITUDE_M}m in {T_TAKEOFF}s. Currently at {alt:.2f}m")

        elif self.state == "SURVEY":
            if self.waypoint_index >= len(self.waypoints):
                self._transition("HOVER")
                self.hover_start_time = None
                self.get_logger().info(f"✅ Survey complete — all {len(self.waypoints)} waypoints reached!")
                self.offboard_setpoint_counter += 1
                return

            tx, ty, tz = self.waypoints[self.waypoint_index]
            self.publish_trajectory_setpoint(tx, ty, tz, velocity=[1.0, 0.0, 0.0])
            dist = self._dist3d(tx, ty, tz)
            wp_elapsed = time.time() - self.waypoint_start_time

            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(
                    f"[SURVEY] WP {self.waypoint_index+1}/{len(self.waypoints)} "
                    f"dist={dist:.2f}m alt={self._alt():.2f}m wp_time={wp_elapsed:.1f}s")

            if dist < POSITION_THRESHOLD_M:
                if self.wp_hover_start is None:
                    self.wp_hover_start = time.time()
                    self.get_logger().info(
                        f"WP {self.waypoint_index+1} reached — hovering {WP_HOVER_DURATION_S}s...")
                elif time.time() - self.wp_hover_start >= WP_HOVER_DURATION_S:
                    self.get_logger().info(
                        f"✅ WP {self.waypoint_index+1} reached in {wp_elapsed:.1f}s")
                    self.waypoint_index += 1
                    self.waypoint_start_time = time.time()
                    self.wp_hover_start = None
            elif wp_elapsed > T_WAYPOINT:
                self._timeout_exit(
                    f"WP {self.waypoint_index+1} not reached in {T_WAYPOINT}s. "
                    f"dist={dist:.2f}m alt={self._alt():.2f}m")

        elif self.state == "HOVER":
            tx, ty, tz = self.waypoints[-1] if self.waypoints else (
                self.home_position[0], self.home_position[1],
                self.home_position[2] - SURVEY_ALTITUDE_M)
            self.publish_trajectory_setpoint(tx, ty, tz)
            alt = self._alt()
            if self.hover_start_time is None:
                if abs(alt - SURVEY_ALTITUDE_M) < ALTITUDE_THRESHOLD_M:
                    self.hover_start_time = time.time()
                    self.get_logger().info(f"✅ HOVER confirmed at {alt:.2f}m. Holding {HOVER_DURATION_S}s...")
            else:
                elapsed_hover = time.time() - self.hover_start_time
                if self.offboard_setpoint_counter % 10 == 0:
                    self.get_logger().info(f"[HOVER] {elapsed_hover:.1f}s/{HOVER_DURATION_S}s alt={alt:.2f}m")
                if elapsed_hover >= HOVER_DURATION_S:
                    self.get_logger().info(
                        f"✅✅ HOVER COMPLETE. Mission done in {self._mission_elapsed():.1f}s total!")
                    self._transition("DONE")

        elif self.state == "DONE":
            if self.waypoints:
                tx, ty, tz = self.waypoints[-1]
                self.publish_trajectory_setpoint(tx, ty, tz)
            if self.offboard_setpoint_counter % 50 == 0:
                self.get_logger().info(
                    f"Mission DONE (total time: {self._mission_elapsed():.1f}s) — Ctrl+C to exit.")

        self.offboard_setpoint_counter += 1


def main(args=None):
    print("Starting control_node v5...")
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
