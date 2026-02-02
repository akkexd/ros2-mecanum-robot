# Week 4-5 Progress Report
## TF2 & URDF Model Creation

**Date:** February 1, 2026

---

## Objectives Completed

- [x] Understand TF2 coordinate transforms
- [x] Create ros_robot_description package
- [x] Create complete URDF model
- [x] Display robot in RViz2
- [x] Verify TF tree is correct

---

## Package Created: ros_robot_description

### Package Structure
```
ros_robot_description/
├── urdf/
│   └── mecanum_robot.urdf    # Robot model
├── launch/
│   └── display.launch.py     # RViz2 visualization
├── rviz/
│   └── display.rviz          # RViz2 config
├── config/
├── meshes/
├── package.xml
└── setup.py
```

### URDF Components

| Link | Description | Geometry |
|------|-------------|----------|
| base_footprint | Ground reference | - |
| base_link | Robot chassis | Box: 300x245x50mm |
| front_left_wheel | Mecanum wheel | Cylinder: r=40mm, h=50mm |
| front_right_wheel | Mecanum wheel | Cylinder: r=40mm, h=50mm |
| back_left_wheel | Mecanum wheel | Cylinder: r=40mm, h=50mm |
| back_right_wheel | Mecanum wheel | Cylinder: r=40mm, h=50mm |
| laser_link | RPLIDAR A2M8 | Cylinder: r=38mm, h=41mm |
| camera_link | RealSense D435 | Box: 25x90x25mm |
| imu_link | IMU sensor | Box: 20x20x5mm |

### TF Tree
```
base_footprint
    └── base_link
        ├── front_left_wheel
        ├── front_right_wheel
        ├── back_left_wheel
        ├── back_right_wheel
        ├── laser_link
        ├── camera_link
        │   ├── camera_depth_frame
        │   │   └── camera_depth_optical_frame
        │   └── camera_color_frame
        │       └── camera_color_optical_frame
        └── imu_link
```

---

## Robot Dimensions

### Chassis (Yahboom Suspension Chassis - Large)
- Length: 300 mm
- Width: 245 mm
- Height: 141 mm
- Weight: 2020 g

### Wheels (80mm Mecanum)
- Diameter: 80 mm (radius: 40 mm)
- Width: ~50 mm
- Wheelbase: 160 mm (center to center)
- Track width: 169 mm (center to center)

### Sensors
- **LiDAR**: RPLIDAR A2M8 (76mm diameter, 41mm height)
- **Camera**: Intel RealSense D435 (90x25x25mm)

---

## Usage

### Display Robot in RViz2
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ros_robot_description display.launch.py
```

### View TF Tree
```bash
ros2 run tf2_tools view_frames
```

---

## Screenshots

Robot displayed in RViz2 with TF frames visible.

---

## Next Steps (Week 6)
- Integrate RPLIDAR A2M8 sensor
- Integrate RealSense D435 camera
- Create robot bringup launch file
