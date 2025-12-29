"""
Top-level launch file for DimenvuePro mapping system.

Launches all required nodes for the mapping pipeline:
- fast_lio node (SLAM with IMU stability monitoring via /mappingState topic)
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

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/shared_data/test_bag',
        description='Path to rosbag for development mode playback'
    )

    # Launch configurations
    development_mode = LaunchConfiguration('development_mode')
    file_format = LaunchConfiguration('file_format')
    artifact_dir = LaunchConfiguration('artifact_dir')
    bag_path = LaunchConfiguration('bag_path')

    # Get package paths
    fast_lio_share = get_package_share_directory('fast_lio')
    fast_lio_config = os.path.join(fast_lio_share, 'config', 'ouster32.yaml')

    # Get the directory where this launch file is located
    current_dir = Path(__file__).parent.absolute()

    # 1. Fast-LIO node (also handles IMU stability monitoring, publishes /mappingState)
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[fast_lio_config],
        name='fastlio_mapping',
        output='screen'
    )

    # 2. Point Cloud Bridge node
    bridge_node = Node(
        package='pointcloud_bridge',
        executable='bridge_node',
        name='bridge_node',
        output='screen',
        parameters=[{
            'pointcloud_topic': '/cloud_registered',
            'pose_topic': '/Odometry',
            'pose_type': 0
        }]
    )

    # 3. Point Cloud Recorder node
    # Note: file_format must be lowercase (ply, pcd, las, laz)
    file_format_lower = PythonExpression(["'", file_format, "'.lower()"])
    recorder_node = Node(
        package='pointcloud_bridge',
        executable='recorder_node_optimized',
        name='recorder_node',
        output='screen',
        parameters=[{
            'pointcloud_topic': '/cloud_registered',
            'pose_topic': '/Odometry',
            'pose_type': 0,
            'artifact_dir': artifact_dir,
            'file_format': file_format_lower
        }]
    )

    # 4. Raw data recorder (ros2 bag record)
    # Using ouster topics for test_bag
    bag_recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', [artifact_dir, '/sensor_raw'],
            '/ouster/points',
            '/ouster/imu'
        ],
        name='bag_recorder',
        output='screen',
        condition=IfCondition(development_mode)
    )

    # 5. Rosbag playback (development mode only)
    bag_playback_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            bag_path,
            '--delay', '2'
        ],
        name='bag_playback',
        output='screen',
        condition=IfCondition(development_mode)
    )

    # 6. Ouster driver (production mode only)
    # Launch ouster_ros driver with sensor IP
    driver_config = current_dir / 'driver_params.yaml'
    ouster_driver = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'ouster_ros', 'driver.launch.py',
            f'params_file:={driver_config}',
            'viz:=false'
        ],
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
        bag_path_arg,

        # Core nodes (always running)
        fast_lio_node,
        bridge_node,
        recorder_node,
        bag_recorder,

        # Conditional nodes based on mode
        bag_playback_node,
        ouster_driver
    ])
