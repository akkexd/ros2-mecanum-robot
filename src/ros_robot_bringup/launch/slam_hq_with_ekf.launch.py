#!/usr/bin/env python3
"""
High-Quality SLAM with EKF Sensor Fusion
Best configuration for mapping!
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
    
    ekf_params_file = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    slam_params_file = os.path.join(pkg_bringup, 'config', 'slam_high_quality.yaml')
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
        
        # Motor Driver (publish_tf=False, EKF publishes it)
        Node(
            package='ros_robot_driver',
            executable='driver_node',
            name='mecanum_driver',
            parameters=[{'publish_tf': True}],
            output='screen'
        ),
        
        # EKF Sensor Fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file],
            remappings=[('odometry/filtered', 'odom')],
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
                'enable_infra1': False,
                'enable_infra2': False,
                'rgb_camera.profile': '640x480x30',
                'depth_module.profile': '640x480x30',
            }],
            output='screen'
        ),
        
        # SLAM Toolbox with HIGH-QUALITY params
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': slam_params_file,
            }.items()
        ),
    ])
