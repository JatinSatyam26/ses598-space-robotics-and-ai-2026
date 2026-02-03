#!/usr/bin/env python3
with open('boustrophedon_controller.py', 'r') as f:
    content = f.read()

# Replace the conditional TURN logic with always-turn logic
old_code = """            # If there is a next waypoint, decide whether we should TURN-in-place
            if self.current_waypoint < len(self.waypoints):
                next_wp = self.waypoints[self.current_waypoint]
                # If Y changes, we are moving to a new row -> do a turn first
                if abs(next_wp[1] - prev_wp[1]) > 1e-3:
                    self.turn_target_angle = math.atan2(
                        next_wp[1] - prev_wp[1],
                        next_wp[0] - prev_wp[0]
                    )
                    self.mode = "TURN"
                    self.prev_linear_error = 0.0
                    self.prev_angular_error = 0.0
                    self.prev_time = self.get_clock().now()"""

new_code = """            # If there is a next waypoint, always turn to align before driving
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
                self.prev_time = self.get_clock().now()"""

content = content.replace(old_code, new_code)

with open('boustrophedon_controller.py', 'w') as f:
    f.write(content)

print("✅ Always-turn fix applied successfully!")
