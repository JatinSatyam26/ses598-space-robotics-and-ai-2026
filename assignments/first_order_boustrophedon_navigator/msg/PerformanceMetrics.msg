# PerformanceMetrics.msg
# Custom message for detailed boustrophedon controller performance tracking

# Cross-track error (distance from desired path)
float64 cross_track_error

# Current velocities
float64 linear_velocity
float64 angular_velocity

# Distance to next waypoint
float64 distance_to_waypoint

# Current waypoint index
int32 current_waypoint_index

# Total number of waypoints
int32 total_waypoints

# Completion percentage (0-100)
float64 completion_percentage

# Controller mode (0 = DRIVE, 1 = TURN)
int32 controller_mode

# Timestamp
builtin_interfaces/Time stamp
