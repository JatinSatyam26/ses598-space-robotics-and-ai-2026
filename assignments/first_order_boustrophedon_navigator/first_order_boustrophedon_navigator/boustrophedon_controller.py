#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim_msgs.msg import Pose
import numpy as np
import math
from collections import deque
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult
import matplotlib.pyplot as plt

class BoustrophedonController(Node):
    def __init__(self):
        super().__init__('lawnmower_controller')
        
        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_linear', 1.0),
                ('Kd_linear', 0.1),
                ('Kp_angular', 1.0),
                ('Kd_angular', 0.1),
                ('spacing', 0.5)
            ]
        )

        # Get initial parameter values
        self.Kp_linear = self.get_parameter('Kp_linear').value
        self.Kd_linear = self.get_parameter('Kd_linear').value
        self.Kp_angular = self.get_parameter('Kp_angular').value
        self.Kd_angular = self.get_parameter('Kd_angular').value
        self.spacing = self.get_parameter('spacing').value
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Create publisher and subscriber
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # Lawnmower pattern parameters
        self.waypoints = self.generate_waypoints()
        self.current_waypoint = 0
        self.mode = "DRIVE"
        self.turn_target_angle = None

        
        # Cross-track error calculation
        self.cross_track_errors = deque(maxlen=1000)  # Store last 1000 errors
        
        # Data for plots
        self.trajectory = []  # To store x, y positions
        self.velocities = []  # To store linear and angular velocities
        
        # State variables
        self.pose = Pose()
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.prev_time = self.get_clock().now()
        
        # Create control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Add publisher for cross-track error
        self.error_pub = self.create_publisher(
            Float64, 
            'cross_track_error', 
            10
        )
        
        self.get_logger().info('Lawnmower controller started')
        self.get_logger().info(f'Following waypoints: {self.waypoints}')

    def generate_waypoints(self):
        waypoints = []
        x_left = 2.0
        x_right = 9.0
        y = 8.0  # start near top

        # Start at the left edge of the first row
        waypoints.append((x_left, y))
        direction = 1  # 1: go to right, -1: go to left

        while True:
            # Drive across the current row
            end_x = x_right if direction == 1 else x_left
            waypoints.append((end_x, y))

            # Step down to the next row (vertical move at the edge)
            next_y = y - self.spacing
            if next_y < 3.0:
                break
            waypoints.append((end_x, next_y))

            # Next row starts where we just stepped down
            y = next_y
            direction *= -1

        return waypoints

    def calculate_cross_track_error(self):
        if self.current_waypoint < 1:
            return 0.0

        start = np.array(self.waypoints[self.current_waypoint - 1])
        end = np.array(self.waypoints[self.current_waypoint])
        pos = np.array([self.pose.x, self.pose.y])

        path_vector = end - start
        path_length = np.linalg.norm(path_vector)
        if path_length < 1e-6:
            return np.linalg.norm(pos - start)

        path_unit = path_vector / path_length
        pos_vector = pos - start

        projection_length = np.dot(pos_vector, path_unit)
        projection_length = max(0, min(path_length, projection_length))
        projected_point = start + projection_length * path_unit

        error_vector = pos - projected_point
        err_norm = np.linalg.norm(error_vector)
        if err_norm < 1e-9:
            error = 0.0
        else:
            error_sign = np.sign(np.cross(path_unit, error_vector / err_norm))
            error = err_norm * error_sign

        self.cross_track_errors.append(abs(error))

        error_msg = Float64()
        error_msg.data = error
        self.error_pub.publish(error_msg)

        return error

    def pose_callback(self, msg):
        self.pose = msg

    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_angle(self, x1, y1, x2, y2):
        return math.atan2(y2 - y1, x2 - x1)

    def control_loop(self):
        # TURN mode: rotate in place until aligned with next segment
        if self.mode == "TURN":
            angle_error = self.turn_target_angle - self.pose.theta
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi

            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = max(min(self.Kp_angular * angle_error, 2.0), -2.0)
            self.velocity_publisher.publish(vel_msg)

            # If aligned, switch back to DRIVE
            if abs(angle_error) < 0.05:
                self.mode = "DRIVE"
                self.turn_target_angle = None
            return

        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('Lawnmower pattern complete')
            if self.cross_track_errors:
                final_avg_error = sum(self.cross_track_errors) / len(self.cross_track_errors)
                self.get_logger().info(f'Final average cross-track error: {final_avg_error:.3f}')
            self.timer.cancel()
            self.plot_data()
            return

        cross_track_error = self.calculate_cross_track_error()

        target_x, target_y = self.waypoints[self.current_waypoint]
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        dt = max(dt, 1e-3)

        distance = self.get_distance(self.pose.x, self.pose.y, target_x, target_y)
        target_angle = self.get_angle(self.pose.x, self.pose.y, target_x, target_y)
        angular_error = target_angle - self.pose.theta

        while angular_error > math.pi:
            angular_error -= 2 * math.pi
        while angular_error < -math.pi:
            angular_error += 2 * math.pi

        linear_error_derivative = (distance - self.prev_linear_error) / dt
        angular_error_derivative = (angular_error - self.prev_angular_error) / dt

        linear_velocity = self.Kp_linear * distance + self.Kd_linear * linear_error_derivative
        angular_velocity = self.Kp_angular * angular_error + self.Kd_angular * angular_error_derivative

        vel_msg = Twist()
        vel_msg.linear.x = min(linear_velocity, 2.0)
        vel_msg.angular.z = max(min(angular_velocity, 2.0), -2.0)
        self.velocity_publisher.publish(vel_msg)

        self.trajectory.append((self.pose.x, self.pose.y))
        self.velocities.append((linear_velocity, angular_velocity))

        self.prev_linear_error = distance
        self.prev_angular_error = angular_error
        self.prev_time = current_time

        if distance < 0.1:
            # Hard stop at waypoint to avoid arcing into the next segment
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self.velocity_publisher.publish(stop)

            # Save the waypoint we just reached
            prev_wp = self.waypoints[self.current_waypoint]

            # Advance to next waypoint
            self.current_waypoint += 1
            self.get_logger().info(f"Reached waypoint {self.current_waypoint}")

            # If there is a next waypoint, decide whether we should TURN-in-place
            if self.current_waypoint < len(self.waypoints):
                next_wp = self.waypoints[self.current_waypoint]

                # Always turn to align with the next segment before driving
                self.turn_target_angle = math.atan2(
                    next_wp[1] - prev_wp[1],
                    next_wp[0] - prev_wp[0]
                )
                self.mode = "TURN"
                self.prev_linear_error = 0.0
                self.prev_angular_error = 0.0
                self.prev_time = self.get_clock().now()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'Kp_linear':
                self.Kp_linear = param.value
            elif param.name == 'Kd_linear':
                self.Kd_linear = param.value
            elif param.name == 'Kp_angular':
                self.Kp_angular = param.value
            elif param.name == 'Kd_angular':
                self.Kd_angular = param.value
            elif param.name == 'spacing':
                self.spacing = param.value
                self.waypoints = self.generate_waypoints()
        return SetParametersResult(successful=True)

    def plot_data(self):
        trajectory = np.array(self.trajectory)
        velocities = np.array(self.velocities)

        # Plot Cross-Track Error
        plt.figure()
        plt.plot(self.cross_track_errors)
        plt.title("Cross-Track Error Over Time")
        plt.xlabel("Time Step")
        plt.ylabel("Error")
        plt.savefig("cross_track_error.png")

        # Plot Trajectory
        plt.figure()
        plt.plot(trajectory[:, 0], trajectory[:, 1], label="Trajectory")
        plt.scatter([wp[0] for wp in self.waypoints], [wp[1] for wp in self.waypoints], c='red', label="Waypoints")
        plt.title("Trajectory Plot")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.savefig("trajectory.png")

        # Plot Velocity Profiles
        plt.figure()
        plt.plot(velocities[:, 0], label="Linear Velocity")
        plt.plot(velocities[:, 1], label="Angular Velocity")
        plt.title("Velocity Profiles")
        plt.xlabel("Time Step")
        plt.ylabel("Velocity")
        plt.legend()
        plt.savefig("velocity_profiles.png")

        self.get_logger().info("Plots saved as PNG files.")

def main(args=None):
    rclpy.init(args=args)
    controller = BoustrophedonController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
