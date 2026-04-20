#!/usr/bin/env python3
"""
Complete SLAM Mapping Launch
Launches: Robot + Sensors + SLAM

IMPORTANT: Uses IncludeLaunchDescription for slam_toolbox to ensure
proper lifecycle node initialization. Running slam_toolbox directly
as a Node() will NOT work - it will stay silent and not publish TF.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_bringup = get_package_share_directory('ros_robot_bringup')
    pkg_description = get_package_share_directory('ros_robot_description')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Config files
    urdf_file = os.path.join(pkg_description, 'urdf', 'mecanum_robot.urdf')
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB1')
    
    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),
        
        # Static transform: odom → base_footprint
        # TEMPORARY: Until real odometry is implemented in Week 9
        # This allows SLAM to work without wheel odometry
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='screen'
        ),
        
        # Motor Driver Node
        Node(
            package='ros_robot_driver',
            executable='driver_node',
            name='mecanum_driver',
            output='screen'
        ),
        
        # RPLIDAR
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': 'laser_link',
                'angle_compensate': True,
            }],
            output='screen'
        ),
        
        # RealSense Camera
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': True,
                'enable_infra2': True,
                'rgb_camera.profile': '640x480x30',
                'depth_module.profile': '848x480x30',
            }],
            output='screen'
        ),

        # ============================================================
        # SLAM Toolbox - MUST use IncludeLaunchDescription!
        # ============================================================
        # slam_toolbox is a LIFECYCLE NODE that requires proper
        # state management (Unconfigured → Inactive → Active).
        # The built-in launch file handles this automatically.
        #
        # DO NOT use Node() directly - it will NOT work!
        # ============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
            }.items()
        ),
    ])
