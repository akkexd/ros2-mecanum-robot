#!/usr/bin/env python3
"""
Mecanum Robot Driver Node with ENCODER-BASED Odometry
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import math
import time

try:
    from Rosmaster_Lib import Rosmaster
    ROSMASTER_AVAILABLE = True
except ImportError:
    ROSMASTER_AVAILABLE = False
    print("WARNING: Rosmaster_Lib not available")


class MecanumDriverNode(Node):
    def __init__(self):
        super().__init__('mecanum_driver')
        
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.14)
        self.declare_parameter('track_width', 0.15)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        
        self.r = self.get_parameter('wheel_radius').value
        self.l = self.get_parameter('wheel_base').value / 2
        self.w = self.get_parameter('track_width').value / 2
        self.serial_port = self.get_parameter('serial_port').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        self.bot = None
        if ROSMASTER_AVAILABLE:
            try:
                self.bot = Rosmaster(com=self.serial_port)
                self.bot.create_receive_threading()
                time.sleep(0.5)
                self.get_logger().info(f'Rosmaster connected on {self.serial_port}')
                try:
                    self.bot.set_auto_report_state(1, 1)
                    self.get_logger().info('Auto report enabled')
                except Exception as e:
                    self.get_logger().warn(f'Could not enable auto report: {e}')
            except Exception as e:
                self.get_logger().error(f'Failed to connect: {e}')
                self.bot = None
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # ACTUAL velocities from encoders
        self.actual_vx = 0.0
        self.actual_vy = 0.0
        self.actual_vth = 0.0
        
        self.last_time = self.get_clock().now()
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        self.wheel_odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.odom_timer = self.create_timer(0.02, self.update_odometry)
        self.imu_timer = self.create_timer(0.02, self.publish_imu)
        self.last_cmd_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('='*50)
        self.get_logger().info('Mecanum Driver - ENCODER-BASED Odometry')
        self.get_logger().info(f'Publishing TF: {self.publish_tf}')
        self.get_logger().info('='*50)

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        
        # MOTORS ONLY - odometry comes from encoders
        motor_vx = -msg.linear.y
        motor_vy = -msg.linear.x
        motor_wz = -msg.angular.z
        
        lw = self.l + self.w
        w1 = (1/self.r) * (motor_vx - motor_vy - lw * motor_wz)
        w2 = (1/self.r) * (motor_vx + motor_vy + lw * motor_wz)
        w3 = (1/self.r) * (motor_vx + motor_vy - lw * motor_wz)
        w4 = (1/self.r) * (motor_vx - motor_vy + lw * motor_wz)
        
        scale = 50
        m1 = max(-100, min(100, int(w1 * scale)))
        m2 = max(-100, min(100, int(w2 * scale)))
        m3 = max(-100, min(100, int(w3 * scale)))
        m4 = max(-100, min(100, int(w4 * scale)))
        
        if self.bot:
            self.bot.set_motor(m1, m2, m3, m4)
            self.get_logger().info(
                f'motors: [{m1}, {m2}, {m3}, {m4}] | '
                f'encoder: vx={self.actual_vx:.3f}, vy={self.actual_vy:.3f}, wz={self.actual_vth:.3f}',
                throttle_duration_sec=0.5
            )

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0 or dt > 0.5:
            return
        
        # READ ACTUAL VELOCITY FROM ENCODERS
        if self.bot:
            try:
                motion = self.bot.get_motion_data()
                if motion and len(motion) >= 3:
                    raw_vx = motion[0] if motion[0] is not None else 0.0
                    raw_vy = motion[1] if motion[1] is not None else 0.0
                    raw_wz = motion[2] if motion[2] is not None else 0.0
                    
                    # Axis transformation: Yahboom → ROS
                    self.actual_vx = raw_vy
                    self.actual_vy = raw_vx
                    self.actual_vth = raw_wz
            except Exception as e:
                self.get_logger().warn(f'Motion error: {e}', throttle_duration_sec=5.0)
        
        vx = self.actual_vx
        vy = self.actual_vy
        vth = self.actual_vth
        
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_th = vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th
        
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        q = self.euler_to_quaternion(0, 0, self.theta)
        
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.01
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.01
        
        self.wheel_odom_pub.publish(odom)
        
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        if self.bot:
            try:
                attitude = self.bot.get_imu_attitude_data()
                if attitude and len(attitude) >= 3:
                    roll = math.radians(attitude[0]) if attitude[0] else 0.0
                    pitch = math.radians(attitude[1]) if attitude[1] else 0.0
                    yaw = math.radians(attitude[2]) if attitude[2] else 0.0
                    imu_msg.orientation = self.euler_to_quaternion(roll, pitch, yaw)
                else:
                    imu_msg.orientation.w = 1.0
                imu_msg.orientation_covariance[0] = 0.01
                imu_msg.orientation_covariance[4] = 0.01
                imu_msg.orientation_covariance[8] = 0.01
            except:
                imu_msg.orientation.w = 1.0
            
            try:
                gyro = self.bot.get_gyroscope_data()
                if gyro and len(gyro) >= 3:
                    imu_msg.angular_velocity.x = math.radians(gyro[0]) if gyro[0] else 0.0
                    imu_msg.angular_velocity.y = math.radians(gyro[1]) if gyro[1] else 0.0
                    imu_msg.angular_velocity.z = math.radians(gyro[2]) if gyro[2] else 0.0
                imu_msg.angular_velocity_covariance[0] = 0.01
                imu_msg.angular_velocity_covariance[4] = 0.01
                imu_msg.angular_velocity_covariance[8] = 0.01
            except:
                pass
            
            try:
                accel = self.bot.get_accelerometer_data()
                if accel and len(accel) >= 3:
                    imu_msg.linear_acceleration.x = accel[0] if accel[0] else 0.0
                    imu_msg.linear_acceleration.y = accel[1] if accel[1] else 0.0
                    imu_msg.linear_acceleration.z = accel[2] if accel[2] else 9.81
                else:
                    imu_msg.linear_acceleration.z = 9.81
                imu_msg.linear_acceleration_covariance[0] = 0.1
                imu_msg.linear_acceleration_covariance[4] = 0.1
                imu_msg.linear_acceleration_covariance[8] = 0.1
            except:
                imu_msg.linear_acceleration.z = 9.81
        else:
            imu_msg.orientation.w = 1.0
            imu_msg.linear_acceleration.z = 9.81
        
        self.imu_pub.publish(imu_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def safety_check(self):
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9 > 0.5:
            if self.bot:
                self.bot.set_motor(0, 0, 0, 0)

    def destroy_node(self):
        if self.bot:
            self.bot.set_motor(0, 0, 0, 0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
