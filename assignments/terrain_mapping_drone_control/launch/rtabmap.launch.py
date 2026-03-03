from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import LogInfo
import os


def generate_launch_description():
    base_path = os.path.join(
        os.environ['HOME'], 'ros2_ws', 'src',
        'terrain_mapping_drone_control', 'terrain_mapping_drone_control'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'
        ),

        ExecuteProcess(
            cmd=['python3', os.path.join(base_path, 'px4_odom_converter.py')],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=[
                '0.1', '0', '0.05',
                '0', '0', '0',
                'base_link',
                'OakD-Lite-Modify/base_link'
            ],
            output='screen'
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_odom_info': False,
                'visual_odometry': False,
                'publish_tf': False,
                'approx_sync': True,
                'approx_sync_max_interval': 0.5,
                'sync_queue_size': 30,
                'topic_queue_size': 30,
                'qos_odom': 2,
                'qos_image': 2,
                'qos_camera_info': 2,
                'database_path': os.path.join(os.environ['HOME'], '.ros', 'rtabmap.db'),
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'true',
                'Rtabmap/DetectionRate': '2.0',
                'Kp/MaxFeatures': '400',
                'Vis/MinInliers': '15',
                'Grid/CellSize': '0.05',
                'Grid/RangeMax': '20.0',
                'Grid/RayTracing': 'true',
                'Grid/FromDepth': 'true',
                'Grid/DepthMin': '0.4',
                'Grid/DepthMax': '20.0',
                'Grid/MaxObstacleHeight': '15.0',
                'Grid/3D': 'true',
                'GridGlobal/MinSize': '0.0',
            }],
            remappings=[
                ('rgb/image', '/drone/front_rgb'),
                ('depth/image', '/drone/front_depth'),
                ('rgb/camera_info', '/drone/front_depth/camera_info'),
                ('odom', '/rtabmap/odom'),
            ]
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'approx_sync': True,
                'qos_odom': 2,
                'qos_image': 2,
                'qos_camera_info': 2,
            }],
            remappings=[
                ('rgb/image', '/drone/front_rgb'),
                ('rgb/camera_info', '/drone/front_depth/camera_info'),
                ('depth/image', '/drone/front_depth'),
                ('odom', '/rtabmap/odom'),
            ]
        ),

        LogInfo(msg='RTAB-Map + PX4 Odom Converter launched'),
    ])
