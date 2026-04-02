#!/usr/bin/env python3
import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, TrajectorySetpoint

SURVEY_ALTITUDE_M    = 5.0
POSITION_THRESHOLD_M = 0.5
ALTITUDE_THRESHOLD_M = 0.5
HOVER_DURATION_S     = 10.0
PREFLIGHT_TICKS      = 10

_SURVEY_OFFSETS = [
    ( 0.0,  0.0), ( 5.0,  0.0), (10.0,  0.0), (15.0,  0.0), (20.0,  0.0),
    (20.0,  5.0), (15.0,  5.0), (10.0,  5.0), ( 5.0,  5.0), ( 0.0,  5.0),
    ( 0.0, 10.0), ( 5.0, 10.0), (10.0, 10.0), (15.0, 10.0),
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
        self.state          = "WAIT_ODOM"
        self.position       = None
        self.home_position  = None
        self.waypoints      = []
        self.waypoint_index = 0
        self.hover_start_time = None
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
        return math.sqrt((self.position[0]-tx)**2 + (self.position[1]-ty)**2 + (self.position[2]-tz)**2)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position=True; msg.velocity=False; msg.acceleration=False; msg.attitude=False; msg.body_rate=False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position=[float(x),float(y),float(z)]; msg.yaw=float(yaw)
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
        self.get_logger().info("Offboard mode command sent")

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def timer_callback(self):
        if self.offboard_setpoint_counter == PREFLIGHT_TICKS:
            self.engage_offboard_mode()
            self.arm()
        self.publish_offboard_control_mode()

        if self.state == "WAIT_ODOM":
            if self.position is not None and self.home_position is not None:
                self.state = "PREFLIGHT"
                self.get_logger().info(f"Odometry received. PREFLIGHT — arm at counter={PREFLIGHT_TICKS}")

        elif self.state == "PREFLIGHT":
            if self.home_position is not None:
                self.publish_trajectory_setpoint(self.home_position[0], self.home_position[1], self.home_position[2])
            if self.offboard_setpoint_counter > PREFLIGHT_TICKS:
                self.state = "TAKEOFF"
                self.get_logger().info(f"Climbing to {SURVEY_ALTITUDE_M}m...")

        elif self.state == "TAKEOFF":
            tz = self.home_position[2] - SURVEY_ALTITUDE_M
            self.publish_trajectory_setpoint(self.home_position[0], self.home_position[1], tz)
            alt = self._alt()
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"[TAKEOFF] {alt:.2f}m / {SURVEY_ALTITUDE_M}m")
            if abs(alt - SURVEY_ALTITUDE_M) < ALTITUDE_THRESHOLD_M:
                self.state = "SURVEY"
                self.waypoint_index = 0
                self.get_logger().info(f"Takeoff complete at {alt:.2f}m. Starting survey ({len(self.waypoints)} waypoints).")

        elif self.state == "SURVEY":
            if self.waypoint_index >= len(self.waypoints):
                self.state = "HOVER"
                self.hover_start_time = None
                self.get_logger().info(f"Survey complete — all {len(self.waypoints)} WPs reached.")
                self.offboard_setpoint_counter += 1
                return
            tx, ty, tz = self.waypoints[self.waypoint_index]
            self.publish_trajectory_setpoint(tx, ty, tz)
            dist = self._dist3d(tx, ty, tz)
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"[SURVEY] WP {self.waypoint_index+1}/{len(self.waypoints)} dist={dist:.2f}m alt={self._alt():.2f}m")
            if dist < POSITION_THRESHOLD_M:
                self.get_logger().info(f"WP {self.waypoint_index+1} reached (dist={dist:.2f}m)")
                self.waypoint_index += 1

        elif self.state == "HOVER":
            tx, ty, tz = self.waypoints[-1] if self.waypoints else (self.home_position[0], self.home_position[1], self.home_position[2] - SURVEY_ALTITUDE_M)
            self.publish_trajectory_setpoint(tx, ty, tz)
            alt = self._alt()
            if self.hover_start_time is None:
                if abs(alt - SURVEY_ALTITUDE_M) < ALTITUDE_THRESHOLD_M:
                    self.hover_start_time = time.time()
                    self.get_logger().info(f"HOVER confirmed at {alt:.2f}m. Holding {HOVER_DURATION_S}s...")
            else:
                elapsed = time.time() - self.hover_start_time
                if self.offboard_setpoint_counter % 10 == 0:
                    self.get_logger().info(f"[HOVER] {elapsed:.1f}s/{HOVER_DURATION_S}s alt={alt:.2f}m")
                if elapsed >= HOVER_DURATION_S:
                    self.get_logger().info("Hover complete. Mission DONE.")
                    self.state = "DONE"

        elif self.state == "DONE":
            if self.waypoints:
                tx, ty, tz = self.waypoints[-1]
                self.publish_trajectory_setpoint(tx, ty, tz)
            if self.offboard_setpoint_counter % 50 == 0:
                self.get_logger().info("Mission DONE — Ctrl+C to exit.")

        self.offboard_setpoint_counter += 1

def main(args=None):
    print("Starting control_node...")
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down control_node.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
