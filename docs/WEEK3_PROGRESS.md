

Progress Report
## ROS2 Robot Driver Implementation

**Date:** January 26, 2026

---

## Objectives

- [x] Create ROS2 package for robot driver
- [x] Implement mecanum kinematics in driver node
- [x] Test robot movement via cmd_vel topic
- [x] Integrate keyboard teleoperation
- [x] Document troubleshooting process

---

## What Was Accomplished

### 1. Created ROS2 Package: `ros_robot_driver`

Package structure:
```
ros_robot_driver/
├── ros_robot_driver/
│   ├── __init__.py
│   └── driver_node.py
├── package.xml
├── setup.py
└── setup.cfg
```

### 2. Implemented Driver Node

The driver node (`driver_node.py`) implements:
- Subscription to `/cmd_vel` topic (geometry_msgs/Twist)
- Mecanum wheel inverse kinematics
- Motor control via Rosmaster_Lib
- Safety timeout (stops motors if no command for 0.5s)
- Debug logging

**Key Code Section - Mecanum Kinematics:**
```python
# Inverse kinematics for mecanum wheels
lw = self.l + self.w  # Combined wheel factor

w1 = (1/self.r) * (vx - vy - lw * wz)  # Front-Left
w2 = (1/self.r) * (vx + vy + lw * wz)  # Front-Right
w3 = (1/self.r) * (vx + vy - lw * wz)  # Rear-Left
w4 = (1/self.r) * (vx - vy + lw * wz)  # Rear-Right
```

### 3. Tested Movement Commands

| Command | Expected | Result |
|---------|----------|--------|
| `linear.x = 0.1` | Forward | ✅ Working |
| `linear.x = -0.1` | Backward | ✅ Working |
| `linear.y = 0.1` | Strafe Left | ✅ Working |
| `linear.y = -0.1` | Strafe Right | ✅ Working |
| `angular.z = 0.5` | Rotate Left | ✅ Working |
| `angular.z = -0.5` | Rotate Right | ✅ Working |
| Combined linear + angular | Forward + Rotate | ✅ Working |

### 4. Keyboard Teleoperation

Successfully tested with `teleop_twist_keyboard`:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Challenges & Solutions

### Challenge 1: Rosmaster_Lib Not Found

**Problem:** ROS2 node couldn't import Rosmaster_Lib
```
ModuleNotFoundError: No module named 'Rosmaster_Lib'
```

**Solution:** Installed Rosmaster_Lib system-wide:
```bash
cd ~/py_install
sudo pip install . --break-system-packages
```

### Challenge 2: Robot Moving in Wrong Directions

**Problem:** Forward command caused strafe, strafe caused forward/backward

**Analysis:**
- `linear.x = 0.1` → Robot strafed right (should go forward)
- `linear.y = 0.1` → Robot went backward (should strafe left)
- Rotation commands worked correctly

**Root Cause:** Robot's physical X/Y axes didn't match ROS convention

**Solution:** Swapped and negated vx/vy in driver_node.py:
```python
# Before (wrong):
vx = msg.linear.x
vy = msg.linear.y

# After (correct for our robot):
vx = -msg.linear.y
vy = -msg.linear.x
```

### Challenge 3: Entry Point Not Registered

**Problem:** `ros2 run ros_robot_driver driver_node` failed

**Solution:** Added entry point to setup.py:
```python
entry_points={
    'console_scripts': [
        'driver_node = ros_robot_driver.driver_node:main',
    ],
},
```

### Challenge 4: "Waiting for subscription" Error

**Problem:** `ros2 topic pub` kept waiting

**Solution:** Must run driver_node in separate terminal first before sending commands.

---

## Files Modified

1. `~/ros2_ws/src/ros_robot_driver/ros_robot_driver/driver_node.py`
   - Created complete driver node with mecanum kinematics
   - Adjusted velocity mapping for robot orientation

2. `~/ros2_ws/src/ros_robot_driver/setup.py`
   - Added entry_points for driver_node

---

## Commands Reference

```bash
# Build package
cd ~/ros2_ws
colcon build --packages-select ros_robot_driver
source install/setup.bash

# Run driver
ros2 run ros_robot_driver driver_node

# Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Manual commands
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
```

---

## Next Steps (Week 4)

1. Implement TF2 coordinate transforms
2. Measure robot dimensions for URDF
3. Create static transform broadcasters
4. Read Modern Robotics Ch 3-4

---

## Lessons Learned

1. **Test incrementally** - Test each component before integrating
2. **Check dependencies** - Verify all Python modules are available system-wide for ROS2
3. **Robot orientation matters** - Physical robot axes may not match ROS convention
4. **Two terminals needed** - Driver must run continuously while sending commands
5. **Rebuild after changes** - Always `colcon build` and `source install/setup.bash`


