#!/usr/bin/env python3
"""
ROS2 Mecanum Robot Driver Node

This node:
1. Subscribes to /cmd_vel (velocity commands from keyboard, Nav2, etc.)
2. Converts velocity to wheel speeds using mecanum kinematics
3. Sends wheel speeds to motors via Rosmaster_Lib (same as Jupyter!)

Location: ~/ros2_ws/src/my_robot_driver/my_robot_driver/driver_node.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

# THIS IS THE SAME LIBRARY YOU USED IN JUPYTER!
from Rosmaster_Lib import Rosmaster

import math


class MecanumDriverNode(Node):
    def __init__(self):
        super().__init__('mecanum_driver')
        
        # ============================================================
        # ROBOT PARAMETERS - MEASURE YOUR ROBOT AND UPDATE THESE!
        # ============================================================
        # Default values - you need to measure your actual robot
        self.declare_parameter('wheel_radius', 0.0325)  # ~65mm diameter wheel = 0.0325m radius
        self.declare_parameter('wheel_base', 0.14)      # front-to-back distance between wheel centers
        self.declare_parameter('track_width', 0.15)     # left-to-right distance between wheel centers
        
        self.r = self.get_parameter('wheel_radius').value
        self.l = self.get_parameter('wheel_base').value / 2   # half of wheelbase
        self.w = self.get_parameter('track_width').value / 2  # half of track width
        
        # ============================================================
        # INITIALIZE ROSMASTER - SAME AS YOUR JUPYTER NOTEBOOK!
        # ============================================================
        self.get_logger().info('Initializing Rosmaster connection...')
        try:
            self.bot = Rosmaster(com="/dev/ttyUSB0")
            self.bot.create_receive_threading()
            self.get_logger().info('Rosmaster connected successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Rosmaster: {e}')
            raise
        
        # ============================================================
        # ROS2 SUBSCRIBER - Listen for velocity commands
        # ============================================================
        # This is where cmd_vel messages come in (from keyboard, Nav2, etc.)
        self.cmd_vel_sub = self.create_subscription(
            Twist,           # Message type
            'cmd_vel',       # Topic name
            self.cmd_vel_callback,  # Function to call when message arrives
            10               # Queue size
        )
        
        # ============================================================
        # ROS2 PUBLISHER - For debugging wheel velocities
        # ============================================================
        self.wheel_pub = self.create_publisher(
            Float32MultiArray,
            'wheel_velocities',
            10
        )
        
        # Store last commanded velocities (for odometry later)
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_wz = 0.0
        
        # Safety timer - stop motors if no commands received for 0.5 seconds
        self.last_cmd_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        # Log startup info
        self.get_logger().info('='*50)
        self.get_logger().info('Mecanum Driver Node Started!')
        self.get_logger().info(f'Wheel radius: {self.r} m')
        self.get_logger().info(f'Wheel base: {self.l*2} m')
        self.get_logger().info(f'Track width: {self.w*2} m')
        self.get_logger().info('='*50)
        self.get_logger().info('Listening for /cmd_vel commands...')
    
    def cmd_vel_callback(self, msg):
        """
        Called whenever a Twist message is received on /cmd_vel
        
        Twist message contains:
        - linear.x  = forward/backward velocity (m/s)
        - linear.y  = left/right velocity (m/s) - mecanum can do this!
        - angular.z = rotation velocity (rad/s)
        """
        # Update last command time (for safety timeout)
        self.last_cmd_time = self.get_clock().now()
        
        # Extract velocities from message
        vx = -msg.linear.y   # forward (m/s)
        vy = -msg.linear.x   # lateral (m/s)
        wz = msg.angular.z  # rotation (rad/s)
        
        # Store for odometry
        self.last_cmd_vx = vx
        self.last_cmd_vy = vy
        self.last_cmd_wz = wz
        
        # ============================================================
        # MECANUM INVERSE KINEMATICS
        # ============================================================
        # Convert robot velocity (vx, vy, wz) to wheel angular velocities
        #
        # For mecanum wheels:
        #   w1 = (1/r) * (vx - vy - (l+w)*wz)  Front-Left
        #   w2 = (1/r) * (vx + vy + (l+w)*wz)  Front-Right
        #   w3 = (1/r) * (vx + vy - (l+w)*wz)  Rear-Left
        #   w4 = (1/r) * (vx - vy + (l+w)*wz)  Rear-Right
        #
        # Note: Signs may need adjustment based on your motor wiring!
        
        lw = self.l + self.w  # Combined factor
        
        # Calculate wheel angular velocities (rad/s)
        w1 = (1/self.r) * (vx - vy - lw * wz)  # Front-Left
        w2 = (1/self.r) * (vx + vy + lw * wz)  # Front-Right
        w3 = (1/self.r) * (vx + vy - lw * wz)  # Rear-Left
        w4 = (1/self.r) * (vx - vy + lw * wz)  # Rear-Right
        
        # ============================================================
        # CONVERT TO MOTOR VALUES
        # ============================================================
        # Rosmaster set_motor() expects values in range ~-1000 to 1000
        # We need to scale from rad/s to motor units
        #
        # IMPORTANT: You may need to adjust this scale factor!
        # If robot moves too slow: increase scale
        # If robot moves too fast: decrease scale
        
        scale = 50  # Adjust this based on testing!
        
        m1 = int(w1 * scale)
        m2 = int(w2 * scale)
        m3 = int(w3 * scale)
        m4 = int(w4 * scale)
        
        # Clamp to valid range (-1000 to 1000)
        m1 = max(-1000, min(1000, m1))
        m2 = max(-1000, min(1000, m2))
        m3 = max(-1000, min(1000, m3))
        m4 = max(-1000, min(1000, m4))
        
        # ============================================================
        # SEND TO MOTORS - SAME AS JUPYTER!
        # ============================================================
        # This is the same function you used in Jupyter Lab!
        self.bot.set_motor(m1, m2, m3, m4)
        
        # Log what we're doing (change 'debug' to 'info' to see always)
        self.get_logger().info(
            f'cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f} -> '
            f'motors: [{m1}, {m2}, {m3}, {m4}]'
        )
        
        # Publish wheel velocities for debugging
        wheel_msg = Float32MultiArray()
        wheel_msg.data = [float(m1), float(m2), float(m3), float(m4)]
        self.wheel_pub.publish(wheel_msg)
    
    def safety_check(self):
        """Stop motors if no command received for 0.5 seconds"""
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > 0.5:
            # No recent command - stop motors
            if self.last_cmd_vx != 0 or self.last_cmd_vy != 0 or self.last_cmd_wz != 0:
                self.bot.set_motor(0, 0, 0, 0)
                self.last_cmd_vx = 0
                self.last_cmd_vy = 0
                self.last_cmd_wz = 0
                self.get_logger().info('Safety stop - no commands received')
    
    def destroy_node(self):
        """Called when node is shutting down - stop motors!"""
        self.get_logger().info('Shutting down - stopping motors...')
        self.bot.set_motor(0, 0, 0, 0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = MecanumDriverNode()
    
    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
