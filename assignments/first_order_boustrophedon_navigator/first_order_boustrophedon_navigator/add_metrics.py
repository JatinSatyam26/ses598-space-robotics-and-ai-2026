#!/usr/bin/env python3
import sys

with open('boustrophedon_controller.py', 'r') as f:
    lines = f.readlines()

# Add import after Float64 import
for i, line in enumerate(lines):
    if 'from std_msgs.msg import Float64' in line:
        lines.insert(i+1, 'from first_order_boustrophedon_navigator.msg import PerformanceMetrics\n')
        break

# Add publisher in __init__ after error_pub
for i, line in enumerate(lines):
    if "'cross_track_error'," in line:
        # Find the closing ) of error_pub
        j = i
        while ')' not in lines[j] or 'self.error_pub' not in ''.join(lines[max(0,j-5):j+1]):
            j += 1
        lines.insert(j+1, '\n        # Publisher for detailed performance metrics (Extra Credit)\n')
        lines.insert(j+2, '        self.metrics_pub = self.create_publisher(\n')
        lines.insert(j+3, '            PerformanceMetrics,\n')
        lines.insert(j+4, "            'performance_metrics',\n")
        lines.insert(j+5, '            10\n')
        lines.insert(j+6, '        )\n')
        break

# Add publishing after velocity_publisher.publish in control_loop
for i, line in enumerate(lines):
    if 'self.velocity_publisher.publish(vel_msg)' in line and 'TURN' not in ''.join(lines[max(0,i-10):i]):
        lines.insert(i+1, '\n        # Publish detailed performance metrics (Extra Credit)\n')
        lines.insert(i+2, '        metrics_msg = PerformanceMetrics()\n')
        lines.insert(i+3, '        metrics_msg.cross_track_error = float(cross_track_error)\n')
        lines.insert(i+4, '        metrics_msg.linear_velocity = float(vel_msg.linear.x)\n')
        lines.insert(i+5, '        metrics_msg.angular_velocity = float(vel_msg.angular.z)\n')
        lines.insert(i+6, '        metrics_msg.distance_to_waypoint = float(distance)\n')
        lines.insert(i+7, '        metrics_msg.current_waypoint_index = self.current_waypoint\n')
        lines.insert(i+8, '        metrics_msg.total_waypoints = len(self.waypoints)\n')
        lines.insert(i+9, '        metrics_msg.completion_percentage = (self.current_waypoint / len(self.waypoints)) * 100.0\n')
        lines.insert(i+10, '        metrics_msg.controller_mode = 1 if self.mode == "TURN" else 0\n')
        lines.insert(i+11, '        metrics_msg.stamp = self.get_clock().now().to_msg()\n')
        lines.insert(i+12, '        self.metrics_pub.publish(metrics_msg)\n')
        break

with open('boustrophedon_controller.py', 'w') as f:
    f.writelines(lines)

print("Custom message code added successfully!")
