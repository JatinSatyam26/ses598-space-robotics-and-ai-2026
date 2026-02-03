with open('boustrophedon_controller.py', 'r') as f:
    lines = f.readlines()

# Find and replace the generate_waypoints function
new_lines = []
skip = False
for i, line in enumerate(lines):
    if 'def generate_waypoints(self):' in line:
        # Add the new function
        new_lines.append(line)
        new_lines.append('        waypoints = []\n')
        new_lines.append('        x_left = 2.0\n')
        new_lines.append('        x_right = 9.0\n')
        new_lines.append('        y = 8.0  # start near top\n')
        new_lines.append('\n')
        new_lines.append('        # Start at the left edge of the first row\n')
        new_lines.append('        waypoints.append((x_left, y))\n')
        new_lines.append('        direction = 1  # 1: go to right, -1: go to left\n')
        new_lines.append('\n')
        new_lines.append('        while True:\n')
        new_lines.append('            # Drive across the current row\n')
        new_lines.append('            end_x = x_right if direction == 1 else x_left\n')
        new_lines.append('            waypoints.append((end_x, y))\n')
        new_lines.append('\n')
        new_lines.append('            # Step down to the next row (vertical move at the edge)\n')
        new_lines.append('            next_y = y - self.spacing\n')
        new_lines.append('            if next_y < 3.0:\n')
        new_lines.append('                break\n')
        new_lines.append('            waypoints.append((end_x, next_y))\n')
        new_lines.append('\n')
        new_lines.append('            # Next row starts where we just stepped down\n')
        new_lines.append('            y = next_y\n')
        new_lines.append('            direction *= -1\n')
        new_lines.append('\n')
        new_lines.append('        return waypoints\n')
        skip = True
    elif skip and ('def calculate_cross_track_error' in line or 'def ' in line and 'generate' not in line):
        skip = False
        new_lines.append(line)
    elif not skip:
        new_lines.append(line)

with open('boustrophedon_controller.py', 'w') as f:
    f.writelines(new_lines)

print("Fixed generate_waypoints!")
