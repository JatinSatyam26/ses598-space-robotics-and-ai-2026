#!/usr/bin/env python3
"""Launch file for midterm project: PX4 SITL + Gazebo planetary terrain + ROS2 bridge."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('midterm_project')
    config_file = os.path.join(pkg_share, 'config', 'mission_config.yaml')

    # Set Gazebo model paths so it can find our terrain model
    gz_model_path = os.path.join(pkg_share, 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        gz_model_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    # Drone starts above terrain center (NED: z=6 means 6m above origin)
    os.environ['PX4_GZ_MODEL_POSE'] = "0,0,6,0,0,0"

    px4_autopilot_path = LaunchConfiguration('px4_autopilot_path')

    # PX4 SITL — use world NAME only (PX4 adds path + .sdf automatically)
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_depth_mono'],
        cwd=px4_autopilot_path,
        additional_env={'PX4_GZ_WORLD': 'planetary_landing'},
        output='screen'
    )

    # Gazebo-ROS2 bridge (same camera/odom topics as Assignment 3)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/mono_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mono_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/model/x500_depth_mono_0/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance',
        ],
        remappings=[
            ('/rgb_camera', '/drone/front_rgb'),
            ('/rgb_camera/camera_info', '/drone/front_rgb/camera_info'),
            ('/depth_camera', '/drone/front_depth'),
            ('/depth_camera/points', '/drone/front_depth/points'),
            ('/camera_info', '/drone/front_depth/camera_info'),
            ('/mono_camera', '/drone/down_mono'),
            ('/mono_camera/camera_info', '/drone/down_mono/camera_info'),
            ('/model/x500_depth_mono_0/odometry_with_covariance', '/fmu/out/vehicle_odometry'),
        ],
        output='screen'
    )

    # Control node with proper params file
    control_node = Node(
        package='midterm_project',
        executable='control_node',
        name='control_node',
        parameters=[config_file, {'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'px4_autopilot_path',
            default_value=os.path.join(
                os.environ.get('HOME', '/home/user'), 'PX4-Autopilot'),
            description='Path to PX4-Autopilot directory'),
        px4_sitl,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=8.0, actions=[control_node]),
    ])
