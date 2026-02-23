#!/usr/bin/env python3
"""
SLAM Launch File
Launches slam_toolbox for mapping
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get config file path
    pkg_bringup = get_package_share_directory('ros_robot_bringup')
    slam_params_file = os.path.join(pkg_bringup, 'config', 'slam_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
        ),
    ])
