#!/usr/bin/env python3
"""
OPTIMIZED Navigation Launch File
- EKF sensor fusion for better odometry
- Auto-activation (no manual lifecycle commands)
- Designed for mecanum robot on carpet
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory('ros_robot_bringup')
    pkg_description = get_package_share_directory('ros_robot_description')
    
    nav2_params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    ekf_params_file = os.path.join(pkg_bringup, 'config', 'ekf_nav.yaml')
    map_file = os.path.join(pkg_bringup, 'maps', 'room_20260217_022826.yaml')
    urdf_file = os.path.join(pkg_description, 'urdf', 'mecanum_robot.urdf')
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB1')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        
        # ============================================================
        # PHASE 1: ROBOT CORE (Immediate)
        # ============================================================
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # Motor Driver - NO TF (EKF publishes TF)
        Node(
            package='ros_robot_driver',
            executable='driver_node',
            name='mecanum_driver',
            parameters=[{'publish_tf': True}],  # EKF publishes TF
            remappings=[('/wheel_odom', '/odom')],
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
                'enable_infra1': False,
                'enable_infra2': False,
            }],
            output='screen'
        ),
        
        # ============================================================
        # PHASE 2: EKF SENSOR FUSION (Delay 2s)
        # Fuses wheel odometry + IMU for accurate orientation
        # ============================================================
        
        # TimerAction(
        #     period=2.0,
        #     actions=[
        #         Node(
        #             package='robot_localization',
        #             executable='ekf_node',
        #             name='ekf_filter_node',
        #             output='screen',
        #             parameters=[ekf_params_file],
        #             remappings=[
        #                 ('/odometry/filtered', '/odom')
        #             ]
        #         ),
        #     ]
        # ),
        
        # ============================================================
        # PHASE 3: MAP SERVER (Delay 4s)
        # ============================================================
        
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}]
                ),
            ]
        ),
        
        # ============================================================
        # PHASE 4: AMCL (Delay 6s)
        # ============================================================
        
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_params_file]
                ),
            ]
        ),
        
        # ============================================================
        # PHASE 5: LOCALIZATION LIFECYCLE (Delay 8s)
        # autostart: True - Auto activates map_server and amcl
        # ============================================================
        
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': ['map_server', 'amcl']
                    }]
                ),
            ]
        ),
        
        # ============================================================
        # PHASE 6: NAVIGATION SERVERS (Delay 12s)
        # ============================================================
        
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params_file]
                ),
                
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params_file]
                ),
                
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params_file]
                ),
                
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[nav2_params_file]
                ),
            ]
        ),
        
        # ============================================================
        # PHASE 7: NAVIGATION LIFECYCLE (Delay 18s)
        # autostart: True - AUTO ACTIVATES ALL NAVIGATION NODES!
        # ============================================================
        
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': True,  # AUTO-START ENABLED!
                        'node_names': [
                            'planner_server',
                            'controller_server',
                            'behavior_server',
                            'bt_navigator'
                        ]
                    }]
                ),
            ]
        ),
    ])
