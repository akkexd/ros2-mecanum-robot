# ROS2 Mecanum Wheel Robot

🤖 An autonomous mecanum wheel mobile robot built with ROS2 Jazzy for TESU Capstone Project.

![Robot Photo](docs/images/robot_photo.jpg)

## Project Overview

This project implements a ROS2-based control system for a mecanum wheel robot, featuring:
- Omnidirectional movement (forward, backward, strafe, rotate)
- Keyboard teleoperation control
- Mecanum wheel inverse kinematics
- Future: SLAM mapping and autonomous navigation

## Hardware

| Component | Model |
|-----------|-------|
| Computer | Raspberry Pi 5 (8GB) |
| OS | Ubuntu 24.04 |
| Motor Controller | Yahboom Expansion Board V3.0 |
| Chassis | Yahboom Mecanum Chassis |
| LiDAR | RPLIDAR A2M8 |
| Depth Camera | Intel RealSense D435 |
| Motors | DC Encoder Motors (4x) |

## Software Requirements

- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.12+
- Rosmaster_Lib (Yahboom driver library)

## Installation

### 1. Clone the repository
```bash
cd ~
git clone git@github.com:akkexd/ros2-mecanum-robot.git
```

### 2. Install ROS2 Dependencies
```bash
sudo apt update
sudo apt install ros-jazzy-rclpy ros-jazzy-std-msgs ros-jazzy-geometry-msgs
sudo apt install ros-jazzy-teleop-twist-keyboard
```

### 3. Install Python Dependencies
```bash
sudo pip install pyserial --break-system-packages
```

### 4. Install Rosmaster_Lib (Yahboom driver)
```bash
cd ~/py_install
sudo pip install . --break-system-packages
```

### 5. Copy to ROS2 workspace and build
```bash
cp -r ~/ros2-mecanum-robot/src/ros_robot_driver ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros_robot_driver
source install/setup.bash
```
## Usage

### Terminal 1: Start the driver node
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros_robot_driver driver_node
```

### Terminal 2: Keyboard control
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Keyboard Controls
```
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i/k = forward/stop
j/l = rotate left/right
u/o = forward + rotate
m/. = backward + rotate

For strafing (hold Shift):
   J    K    L
   (left) (stop) (right)
```

### Manual velocity commands
```bash
# Forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

# Rotate
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Forward + Rotate
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
```

## Project Structure

```
ros2-mecanum-robot/
├── README.md
├── docs/
│   ├── SETUP.md
│   ├── TROUBLESHOOTING.md
│   ├── WEEK3_PROGRESS.md
│   └── images/
├── src/
│   └── ros_robot_driver/
│       ├── ros_robot_driver/
│       │   ├── __init__.py
│       │   └── driver_node.py
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── scripts/
├── config/
└── maps/
```

## Progress

- [x] Week 1: Hardware setup, motor testing with Rosmaster_Lib
- [x] Week 2: ROS2 workspace setup, package creation
- [x] Week 3: Driver node implementation, keyboard teleoperation
- [ ] Week 4: TF2 coordinate transforms
- [ ] Week 5: URDF model creation
- [ ] Week 6: Sensor integration (LiDAR, Camera)
- [ ] Week 7-8: SLAM mapping
- [ ] Week 9: Odometry implementation
- [ ] Week 10-11: Nav2 navigation
- [ ] Week 12: Capstone completion

## Troubleshooting

See [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) for common issues and solutions.

## Author
Aung Khant Ko
