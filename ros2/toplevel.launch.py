"""
Top-level launch file for DimenvuePro mapping system.

Launches all required nodes for the mapping pipeline:
- fast_lio node (SLAM)
- imu_monitor node (initialization tracking)
- point_cloud_bridge node (streaming)
- point_cloud_recorder node (saving)
- raw data recorder (rosbag)
- ouster driver (production mode) OR rosbag playback (development mode)
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with all mapping nodes."""

    # Declare launch arguments
    development_mode_arg = DeclareLaunchArgument(
        'development_mode',
        default_value='false',
        description='Run in development mode (bag playback) or production mode (real sensor)'
    )

    file_format_arg = DeclareLaunchArgument(
        'file_format',
        default_value='PLY',
        description='Output file format for point cloud data (PLY, PCD, LAS, LAZ)'
    )

    artifact_dir_arg = DeclareLaunchArgument(
        'artifact_dir',
        default_value='',
        description='Directory to store all mapping artifacts'
    )

    # Launch configurations
    development_mode = LaunchConfiguration('development_mode')
    file_format = LaunchConfiguration('file_format')
    artifact_dir = LaunchConfiguration('artifact_dir')

    # Get package paths
    fast_lio_share = get_package_share_directory('fast_lio')
    fast_lio_config = os.path.join(fast_lio_share, 'config', 'ouster32.yaml')

    # Get the directory where this launch file is located (for imu_monitor.py)
    current_dir = Path(__file__).parent.absolute()
    imu_monitor_script = str(current_dir / 'imu_monitor.py')

    # 1. Fast-LIO node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[fast_lio_config]
    )

    # 2. IMU Monitor node (as ExecuteProcess for command-line args)
    imu_topic_expr = PythonExpression([
        '"', '/imu/data', '" if "', development_mode, '" == "true" else "', '/ouster/imu', '"'
    ])

    imu_monitor_node = ExecuteProcess(
        cmd=[
            'python3',
            imu_monitor_script,
            '--imu-topic', imu_topic_expr,
            '--track-duration', '10.0',
            '--result-path', [artifact_dir, '/imu_stabilization_status.txt']
        ],
        name='imu_monitor',
        output='screen'
    )

    # 3. Point Cloud Bridge node
    bridge_node = Node(
        package='point_cloud_bridge',
        executable='bridge_node',
        name='bridge_node',
        output='screen',
        parameters=[{
            'pointcloud_topic': '/cloud_registered_body',
            'pose_topic': '/Odometry',
            'pose_type': 0
        }]
    )

    # 4. Point Cloud Recorder node
    recorder_node = Node(
        package='point_cloud_bridge',
        executable='recorder_node',
        name='recorder_node',
        output='screen',
        parameters=[{
            'pointcloud_topic': '/cloud_registered_body',
            'pose_topic': '/Odometry',
            'pose_type': 0,
            'artifact_dir': artifact_dir,
            'file_format': file_format
        }]
    )

    # 5. Raw data recorder (ros2 bag record)
    # Build topic list based on mode
    bag_topics_dev = [
        '/os1_cloud_node/points',
        '/imu/data',
        '/Odometry'
    ]

    bag_topics_prod = [
        '/os1_cloud_node/points',
        '/ouster/imu',
        '/Odometry'
    ]

    # Development mode bag recorder
    bag_recorder_dev = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', [artifact_dir, '/sensor_raw'],
            '/os1_cloud_node/points',
            '/imu/data',
            '/Odometry'
        ],
        name='bag_recorder',
        output='screen',
        condition=IfCondition(development_mode)
    )

    # Production mode bag recorder
    bag_recorder_prod = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', [artifact_dir, '/sensor_raw'],
            '/os1_cloud_node/points',
            '/ouster/imu',
            '/Odometry'
        ],
        name='bag_recorder',
        output='screen',
        condition=UnlessCondition(development_mode)
    )

    # 6. Rosbag playback (development mode only)
    bag_playback_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            '/shared_data/office_sim_bag/',
            '--delay', '2'
        ],
        name='bag_playback',
        output='screen',
        condition=IfCondition(development_mode)
    )

    # 7. Ouster driver node (production mode only)
    # Note: You may need to adjust this based on your actual ouster driver package
    ouster_driver_node = Node(
        package='ros2_ouster',
        executable='ouster_driver',
        name='ouster_driver',
        output='screen',
        condition=UnlessCondition(development_mode)
    )

    # Build launch description
    return LaunchDescription([
        # Launch arguments
        development_mode_arg,
        file_format_arg,
        artifact_dir_arg,

        # Core nodes (always running)
        fast_lio_node,
        imu_monitor_node,
        bridge_node,
        recorder_node,

        # Conditional nodes based on mode
        bag_recorder_dev,
        bag_recorder_prod,
        bag_playback_node,
        ouster_driver_node
    ])
