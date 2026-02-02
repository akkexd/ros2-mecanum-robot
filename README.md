# ROS2 Mecanum Robot

🤖 Autonomous mecanum wheel mobile robot with ROS2 Jazzy - TESU Capstone Project

## Project Overview

This project implements a ROS2-based control system for a mecanum wheel robot featuring:
- Omnidirectional movement (forward, backward, strafe, rotate)
- Keyboard teleoperation
- URDF robot model with TF transforms
- Future: SLAM mapping and autonomous navigation

## Hardware

| Component | Model |
|-----------|-------|
| Computer | Raspberry Pi 5 (8GB) |
| OS | Ubuntu 24.04 |
| Motor Controller | Yahboom Expansion Board |
| Chassis | Yahboom Mecanum Chassis (L) |
| LiDAR | RPLIDAR A2M8 |
| Depth Camera | Intel RealSense D435 |
| Motors | MD520Z56_12V with 1:56 reduction |
| Wheels | 80mm Mecanum wheels |

## Packages

| Package | Description |
|---------|-------------|
| `ros_robot_driver` | Motor control, mecanum kinematics |
| `ros_robot_description` | URDF model, TF transforms |

## Installation

### 1. Clone Repository
```bash
cd ~
git clone git@github.com:YOUR-USERNAME/ros2-mecanum-robot.git
```

### 2. Install Dependencies
```bash
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-teleop-twist-keyboard
sudo apt install ros-jazzy-rviz2

# Install Rosmaster_Lib
cd ~/py_install
sudo pip install . --break-system-packages
```

### 3. Build
```bash
cp -r ~/ros2-mecanum-robot/src/* ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Usage

### Drive Robot with Keyboard
```bash
# Terminal 1: Start driver
ros2 run ros_robot_driver driver_node

# Terminal 2: Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### View Robot Model in RViz2
```bash
ros2 launch ros_robot_description display.launch.py
```

## Project Structure
```
ros2-mecanum-robot/
├── README.md
├── docs/
│   ├── TROUBLESHOOTING.md
│   ├── WEEK3_PROGRESS.md
│   └── WEEK4_5_PROGRESS.md
├── src/
│   ├── ros_robot_driver/
│   │   └── ros_robot_driver/
│   │       └── driver_node.py
│   └── ros_robot_description/
│       ├── urdf/
│       │   └── mecanum_robot.urdf
│       └── launch/
│           └── display.launch.py
└── scripts/
```

## Progress

- [x] Week 1: Hardware setup, motor testing
- [x] Week 2: ROS2 workspace, package creation
- [x] Week 3: Driver node, keyboard teleoperation
- [x] Week 4-5: TF2 transforms, URDF model
- [ ] Week 6: Sensor integration (LiDAR, Camera)
- [ ] Week 7-8: SLAM mapping
- [ ] Week 9: Odometry
- [ ] Week 10-11: Nav2 navigation
- [ ] Week 12: Capstone completion

## Author

**Aung**
- TESU Computer Science Student
- Email: aungkko.edu@gmail.com

## License

MIT License
