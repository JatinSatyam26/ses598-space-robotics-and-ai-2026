"""
Phase 31-34: PX4 + Gazebo launch file
KEY LESSONS FROM TRANSCRIPT:
- Pass world NAME only to PX4, not full path (PX4 prepends its own dir + .sdf)
- TimerAction sequencing: PX4 first, bridge at 3s, nodes at 8s
- DDS agent and QGC are launched in separate terminals (not here)
- QoS must be BEST_EFFORT + TRANSIENT_LOCAL
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Phase 31: PX4 SITL — world name only, not full path
    px4 = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'cd ~/PX4-Autopilot && PX4_GZ_WORLD=planetary_landing '
            './build/px4_sitl_default/bin/px4 '
            './build/px4_sitl_default/etc -s etc/init.d-posix/rcS'
        ],
        output='screen'
    )

    # Phase 32: Gazebo-ROS2 bridge — all required topics
    bridge = TimerAction(period=3.0, actions=[
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/drone/front_rgb@sensor_msgs/msg/Image[gz.msgs.Image',
                '/drone/front_depth@sensor_msgs/msg/Image[gz.msgs.Image',
                '/drone/front_rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/drone/front_depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/fmu/out/vehicle_odometry@px4_msgs/msg/VehicleOdometry[gz.msgs.Odometry',
            ],
            output='screen'
        )
    ])

    # Phase 33-34: control node at 8s with config
    control = TimerAction(period=8.0, actions=[
        Node(
            package='midterm_project',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[
                os.path.join(
                    os.path.expanduser('~'),
                    'ses598-space-robotics-and-ai-2026',
                    'midterm_project', 'config', 'mission_config.yaml'
                )
            ],
            remappings=[
                ('/drone/front_rgb', '/drone/front_rgb'),
                ('/drone/front_depth', '/drone/front_depth'),
            ]
        )
    ])

    return LaunchDescription([px4, bridge, control])
