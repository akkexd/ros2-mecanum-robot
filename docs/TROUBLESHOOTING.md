# Troubleshooting Guide

## Common Issues and Solutions

---

### Issue 1: "ModuleNotFoundError: No module named 'Rosmaster_Lib'"

**Symptom:**
```
ModuleNotFoundError: No module named 'Rosmaster_Lib'
```

**Solution:**
```bash
# Install Rosmaster_Lib system-wide
cd ~/py_install
sudo pip install . --break-system-packages

# Verify installation
python3 -c "from Rosmaster_Lib import Rosmaster; print('OK')"
```

---

### Issue 2: "Permission denied: /dev/ttyUSB0"

**Symptom:**
```
Permission denied: '/dev/ttyUSB0'
```

**Solution:**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Or set permissions directly
sudo chmod 666 /dev/ttyUSB0

# Verify
ls -la /dev/ttyUSB*
```

---

### Issue 3: "Waiting for at least 1 matching subscription(s)..."

**Symptom:**
When running `ros2 topic pub`, it keeps waiting.

**Cause:**
The driver node is not running.

**Solution:**
Run the driver node in Terminal 1 first:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros_robot_driver driver_node
```

Then send commands in Terminal 2.

---

### Issue 4: Robot Moves in Wrong Direction

**Symptom:**
- Forward command moves robot backward
- Strafe left moves robot right
- Commands are swapped

**Cause:**
Robot's physical orientation doesn't match ROS convention.

**Solution:**
Edit `driver_node.py` and adjust the velocity mapping:

```python
# In cmd_vel_callback function, change:
vx = msg.linear.x
vy = msg.linear.y

# To (adjust signs as needed for your robot):
vx = -msg.linear.y   # Swap and negate
vy = -msg.linear.x   # Swap and negate
wz = msg.angular.z   # Keep as-is
```

Rebuild after changes:
```bash
cd ~/ros2_ws
colcon build --packages-select ros_robot_driver
source install/setup.bash
```

---

### Issue 5: Teleop Keyboard Not Responding

**Symptom:**
Pressing keys in teleop_twist_keyboard does nothing.

**Solution:**
1. Make sure driver_node is running in another terminal
2. Make sure the teleop terminal has focus (click on it)
3. Check if topics are connected:
```bash
ros2 topic list
ros2 topic echo /cmd_vel
```

---

### Issue 6: Build Error - Entry Point Not Found

**Symptom:**
```
ros2 run ros_robot_driver driver_node
No executable found
```

**Solution:**
Check `setup.py` has the entry point:
```python
entry_points={
    'console_scripts': [
        'driver_node = ros_robot_driver.driver_node:main',
    ],
},
```

Rebuild:
```bash
cd ~/ros2_ws
colcon build --packages-select ros_robot_driver
source install/setup.bash
```

---

### Issue 7: Bash Special Character Error

**Symptom:**
```
-bash: !': event not found
```

**Cause:**
Bash interprets `!` as a special character.

**Solution:**
Use single quotes instead of double quotes:
```bash
# Instead of:
python3 -c "print('Hello!')"

# Use:
python3 -c 'print("Hello")'
```

---

## Diagnostic Commands

```bash
# Check if ROS2 is sourced
echo $ROS_DISTRO
# Should show: jazzy

# List running nodes
ros2 node list

# List topics
ros2 topic list

# Monitor cmd_vel
ros2 topic echo /cmd_vel

# Check serial ports
ls -la /dev/ttyUSB*

# Test Rosmaster_Lib
python3 -c "from Rosmaster_Lib import Rosmaster; print('OK')"

