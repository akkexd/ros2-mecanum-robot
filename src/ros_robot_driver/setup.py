from setuptools import find_packages, setup

package_name = 'ros_robot_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2mobilerobot',
    maintainer_email='ros2mobilerobot@todo.todo',
    description='Mecanum robot driver for ROS2',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'driver_node = ros_robot_driver.driver_node:main',
            'rotate_after_goal = ros_robot_driver.rotate_after_goal:main',
            'goal_manager = ros_robot_driver.goal_manager:main',
            'goal_manager_v4 = ros_robot_driver.goal_manager_v4:main',
            'measure_doorways = ros_robot_driver.measure_doorways:main',
        ],
    },
)
