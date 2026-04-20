#!/usr/bin/env python3
"""
SLAM Mapping Launch — EKF + High Quality
Based on existing slam_hq_with_ekf.launch with fixes:
  - publish_tf: False (EKF publishes TF, not driver)
  - ekf_nav.yaml (current working EKF config)
  - IncludeLaunchDescription for slam_toolbox (lifecycle node)
  - slam_params_file pointing to slam_mapping_params.yaml

Drive with: ros2 run teleop_twist_keyboard teleop_twist_keyboard
Save map:   ros2 run nav2_map_server map_saver_cli -f ~/maps/apartment_full
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_bringup = get_package_share_directory('ros_robot_bringup')
    pkg_description = get_package_share_directory('ros_robot_description')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    ekf_params_file = os.path.join(pkg_bringup, 'config', 'ekf_nav.yaml')
    slam_params_file = os.path.join(pkg_bringup, 'config', 'slam_mapping_params.yaml')
    urdf_file = os.path.join(pkg_description, 'urdf', 'mecanum_robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB1')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),

        # ============================================================
        # ROBOT CORE
        # ============================================================

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),

        
        # Motor Driver — publish_tf=False because EKF publishes odom→base_footprint
        # Motor Driver — publish_tf=True for SLAM (no EKF needed)
        Node(
            package='ros_robot_driver',
            executable='driver_node',
            name='mecanum_driver',
            parameters=[{'publish_tf': False}],
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

        # RealSense Camera (for documentation video)
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
        # EKF SENSOR FUSION (Delay 2s)
        # Fuses wheel_odom + IMU for accurate odom→base_footprint TF
        # ============================================================

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[ekf_params_file],
                    remappings=[
                        ('/odometry/filtered', '/odom')
                    ]
                ),
            ]
        ),

        # ============================================================
        # SLAM TOOLBOX (Delay 4s)
        # MUST use IncludeLaunchDescription — slam_toolbox is a
        # lifecycle node that requires proper state management.
        # Node() directly will NOT work (stays silent, no TF).
        # ============================================================

        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'slam_params_file': slam_params_file,
                    }.items()
                ),
            ]
        ),
    ])