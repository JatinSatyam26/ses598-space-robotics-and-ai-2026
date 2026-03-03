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

        # ── 1. PX4 Odometry Converter ─────────────────────────────────
        # Converts PX4 VehicleOdometry → nav_msgs/Odometry + TF
        # Uses camera timestamps for perfect sync with RTAB-Map
        ExecuteProcess(
            cmd=['python3', os.path.join(base_path, 'px4_odom_converter.py')],
            output='screen',
        ),

        # ── 2. Static TF: base_link → camera_link ────────────────────
        # Camera is mounted ~10cm forward, ~5cm above drone center
        # Using identity rotation (camera aligned with body frame)
        # RTAB-Map will handle the optical frame internally
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_link',
            arguments=[
                '0.1', '0', '0.05',    # translation x, y, z
                '0', '0', '0',          # rotation r, p, y (no rotation)
                'base_link',             # parent frame
                'camera_link'            # child frame
            ],
            output='screen'
        ),

        # ── 3. RTAB-Map SLAM Node ─────────────────────────────────────
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),

                # Frame configuration
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',

                # Input configuration
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_odom_info': False,
                'visual_odometry': False,  # Using PX4 odometry
                'approx_sync': True,
                'approx_sync_max_interval': 0.1,  # 100ms sync window
                'queue_size': 30,

                # Database — fresh each run
                'database_path': os.path.join(os.environ['HOME'], '.ros', 'rtabmap.db'),
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'true',

                # Detection / loop closure
                'Rtabmap/DetectionRate': '2.0',    # Process 2 frames/sec
                'Kp/MaxFeatures': '400',
                'Vis/MinInliers': '15',

                # Grid / mapping
                'Grid/CellSize': '0.05',
                'Grid/RangeMax': '20.0',
                'Grid/RayTracing': 'true',
                'Grid/FromDepth': 'true',

                # Depth filtering
                'Grid/DepthMin': '0.4',
                'Grid/DepthMax': '20.0',
                'Grid/MaxObstacleHeight': '15.0',

                # 3D map
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

        # ── 4. Point cloud node (for visualization / mesh export) ─────
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyzrgb',
            name='point_cloud_xyzrgb',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'decimation': 4,
                'voxel_size': 0.05,
                'approx_sync': True,
                'approx_sync_max_interval': 0.1,
            }],
            remappings=[
                ('rgb/image', '/drone/front_rgb'),
                ('depth/image', '/drone/front_depth'),
                ('rgb/camera_info', '/drone/front_depth/camera_info'),
                ('cloud', '/rtabmap/cloud_map'),
            ]
        ),

        LogInfo(msg='RTAB-Map + PX4 Odom Converter launched for 3D mapping'),
    ])
