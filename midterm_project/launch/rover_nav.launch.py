"""
rover_nav.launch.py — Spawn Mars rover + bridge + Nav2

Assumes Gazebo + PX4 are already running (planetary_landing.launch.py).

Launch sequence:
  t=0s   Spawn rover SDF into running Gazebo world
  t=3s   ros_gz_bridge for cmd_vel, odom, joint_states, tf
  t=5s   robot_state_publisher (URDF-less, uses joint_states)
  t=6s   Nav2 bringup with static costmap from ~/midterm_reconstruction/

Usage:
  ros2 launch midterm_project rover_nav.launch.py
  ros2 launch midterm_project rover_nav.launch.py \
      costmap_yaml:=/path/to/costmap.yaml \
      spawn_x:=8.0 spawn_y:=0.0
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, GroupAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('midterm_project')

    # ── Launch arguments ────────────────────────────────────────────────────
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x', default_value='8.0',
        description='Rover spawn X position in world frame')
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='0.0',
        description='Rover spawn Y position in world frame')
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z', default_value='0.2',
        description='Rover spawn Z position in world frame')

    costmap_yaml_arg = DeclareLaunchArgument(
        'costmap_yaml',
        default_value=str(Path.home() / 'midterm_reconstruction' / 'costmap.yaml'),
        description='Path to Nav2 map_server costmap.yaml')

    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='true',
        description='Whether to launch Nav2 stack')

    spawn_x    = LaunchConfiguration('spawn_x')
    spawn_y    = LaunchConfiguration('spawn_y')
    spawn_z    = LaunchConfiguration('spawn_z')
    costmap_yaml = LaunchConfiguration('costmap_yaml')
    use_nav2   = LaunchConfiguration('use_nav2')

    # ── Model SDF path ──────────────────────────────────────────────────────
    rover_sdf = str(
        Path.home() /
        'ses598-space-robotics-and-ai-2026' /
        'midterm_project' / 'models' / 'mars_rover' / 'model.sdf'
    )

    # ── 1. Spawn rover into Gazebo ──────────────────────────────────────────
    spawn_rover = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'mars_rover',
            '-file', rover_sdf,
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
        ],
        output='screen',
        name='spawn_mars_rover',
    )

    # ── 2. ros_gz_bridge: Gazebo ↔ ROS2 for rover topics ───────────────────
    #
    # Topic mapping (Gazebo model topic → ROS2 topic):
    #   /model/mars_rover/cmd_vel   ← /rover/cmd_vel   (Twist, bidirectional @)
    #   /model/mars_rover/odometry  → /rover/odom       (Odometry, gz→ros [)
    #   /model/mars_rover/tf        → /tf               (TFMessage, gz→ros [)
    #   /world/planetary_landing/model/mars_rover/joint_state → /joint_states
    #
    # Bridge format: <ros_topic>@<ros_type>[<gz_type>   = gz→ros (subscribe)
    #                <ros_topic>@<ros_type>]<gz_type>   = ros→gz (publish)
    #                <ros_topic>@<ros_type>@<gz_type>   = bidirectional
    rover_bridge = TimerAction(period=3.0, actions=[
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                # cmd_vel: ROS→Gazebo (rover listens to cmd_vel in gz)
                '/model/mars_rover/cmd_vel'
                '@geometry_msgs/msg/Twist'
                ']gz.msgs.Twist',
                # odom: Gazebo→ROS
                '/model/mars_rover/odometry'
                '@nav_msgs/msg/Odometry'
                '[gz.msgs.Odometry',
                # TF from DiffDrive plugin
                '/model/mars_rover/tf'
                '@tf2_msgs/msg/TFMessage'
                '[gz.msgs.Pose_V',
                # Joint states
                '/world/planetary_landing/model/mars_rover/joint_state'
                '@sensor_msgs/msg/JointState'
                '[gz.msgs.Model',
                # Remap args
                '--ros-args',
                '-r', '/model/mars_rover/cmd_vel:=/rover/cmd_vel',
                '-r', '/model/mars_rover/odometry:=/rover/odom',
                '-r', '/model/mars_rover/tf:=/tf',
                '-r', '/world/planetary_landing/model/mars_rover/joint_state:=/joint_states',
            ],
            output='screen',
            name='rover_bridge',
        )
    ])

    # ── 3. robot_state_publisher ─────────────────────────────────────────────
    # Minimal URDF-less RSP: broadcasts joint transforms from /joint_states.
    # We use a minimal robot description (just the frame relationships).
    robot_description = """<?xml version="1.0"?>
<robot name="mars_rover">
  <link name="base_link"/>
  <link name="left_wheel"/>
  <link name="right_wheel"/>
  <link name="front_left_wheel"/>
  <link name="front_right_wheel"/>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="left_wheel"/>
    <origin xyz="-0.18 0.17 -0.075" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="right_wheel"/>
    <origin xyz="-0.18 -0.17 -0.075" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="front_left_wheel"/>
    <origin xyz="0.18 0.17 -0.075" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="front_right_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="front_right_wheel"/>
    <origin xyz="0.18 -0.17 -0.075" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>"""

    rsp = TimerAction(period=5.0, actions=[
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rover_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
        )
    ])

    # ── 4. Nav2 bringup ──────────────────────────────────────────────────────
    nav2_params_file = str(
        Path.home() /
        'ses598-space-robotics-and-ai-2026' /
        'midterm_project' / 'config' / 'nav2_params.yaml'
    )

    nav2 = TimerAction(period=6.0, actions=[
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
                f'params_file:={nav2_params_file}',
                'use_sim_time:=true',
                'autostart:=true',
                f'map:={costmap_yaml}',
            ],
            output='screen',
            name='nav2_bringup',
        )
    ])

    return LaunchDescription([
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        costmap_yaml_arg,
        use_nav2_arg,
        spawn_rover,
        rover_bridge,
        rsp,
        nav2,
    ])
