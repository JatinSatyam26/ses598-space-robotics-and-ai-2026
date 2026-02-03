#!/usr/bin/env python3
with open('boustrophedon_controller.py', 'r') as f:
    content = f.read()

# 1. Add import
content = content.replace(
    'from std_msgs.msg import Float64',
    'from std_msgs.msg import Float64\nfrom first_order_boustrophedon_navigator.msg import PerformanceMetrics'
)

# 2. Add publisher
content = content.replace(
    "            'cross_track_error', \n            10\n        )",
    "            'cross_track_error', \n            10\n        )\n        self.metrics_pub = self.create_publisher(PerformanceMetrics, 'performance_metrics', 10)"
)

# 3. Add publishing in control_loop
lines = content.split('\n')
new_lines = []
for i, line in enumerate(lines):
    new_lines.append(line)
    if 'self.velocity_publisher.publish(vel_msg)' in line:
        context = '\n'.join(lines[max(0,i-15):i])
        if 'if self.mode == "TURN"' not in context:
            new_lines.append('')
            new_lines.append('        # Extra Credit: Performance Metrics')
            new_lines.append('        m = PerformanceMetrics()')
            new_lines.append('        m.cross_track_error = float(cross_track_error)')
            new_lines.append('        m.linear_velocity = float(vel_msg.linear.x)')
            new_lines.append('        m.angular_velocity = float(vel_msg.angular.z)')
            new_lines.append('        m.distance_to_waypoint = float(distance)')
            new_lines.append('        m.current_waypoint_index = self.current_waypoint')
            new_lines.append('        m.total_waypoints = len(self.waypoints)')
            new_lines.append('        m.completion_percentage = (self.current_waypoint / len(self.waypoints)) * 100.0')
            new_lines.append('        m.controller_mode = 1 if self.mode == "TURN" else 0')
            new_lines.append('        m.stamp = self.get_clock().now().to_msg()')
            new_lines.append('        self.metrics_pub.publish(m)')

with open('boustrophedon_controller.py', 'w') as f:
    f.write('\n'.join(new_lines))

print("Fixed!")
