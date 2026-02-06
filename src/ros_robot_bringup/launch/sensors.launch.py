#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB1')
    
    pkg_description = get_package_share_directory('ros_robot_description')
    urdf_file = os.path.join(pkg_description, 'urdf', 'mecanum_robot.urdf')
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
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
        
        # RPLIDAR using sllidar_ros2
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
        
        # RealSense D435
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'rgb_camera.profile': '640x480x30',
                'depth_module.profile': '640x480x30',
            }],
            output='screen'
        ),
    ])
