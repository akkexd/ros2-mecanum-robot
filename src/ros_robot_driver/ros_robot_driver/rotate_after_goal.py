#!/usr/bin/env python3
"""
Rotate After Goal — Phase 2 Heading Alignment for Mecanum Navigation

v3 improvements:
  - Kickstart pulse (0.65 rad/s for 0.5s) to break carpet static friction
  - Stall detection: aborts if <3 deg progress in 3 seconds
  - LIDAR safety: aborts rotation if obstacle < 0.20m nearby

Axis conventions (Yahboom ROSMASTER X3):
  Driver: motor_wz = -msg.angular.z
  IMU publisher: yaw = -math.radians(attitude[2])
  Both inversions mean:
    positive cmd.angular.z -> driver negates -> physical CW (right)
    negative cmd.angular.z -> driver negates -> physical CCW (left)
  IMU delta must be NEGATED to match ROS convention.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, LaserScan
from action_msgs.msg import GoalStatusArray
from tf2_ros import Buffer, TransformListener
import rclpy.time
import rclpy.duration
import math
import numpy as np


class State:
    IDLE = 0
    SETTLING = 1
    KICKSTART = 2   # NEW: high-power pulse to break static friction
    ROTATING = 3
    STALLED = 4     # NEW: rotation failed


class RotateAfterGoal(Node):
    def __init__(self):
        super().__init__('rotate_after_goal')

        # --- Parameters ---
        self.declare_parameter('kickstart_speed', 0.65)   # rad/s -> motor ~72
        self.declare_parameter('kickstart_duration', 0.5)  # seconds
        self.declare_parameter('rotation_speed', 0.50)     # rad/s -> motor ~56
        self.declare_parameter('yaw_tolerance', 0.15)      # rad = 8.6 deg
        self.declare_parameter('settle_time', 0.5)
        self.declare_parameter('timeout', 30.0)
        self.declare_parameter('stall_check_time', 3.0)    # seconds before stall check
        self.declare_parameter('stall_threshold', 3.0)     # degrees minimum progress
        self.declare_parameter('obstacle_distance', 0.20)  # meters - abort if closer

        self.kickstart_speed = self.get_parameter('kickstart_speed').value
        self.kickstart_duration = self.get_parameter('kickstart_duration').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.settle_time = self.get_parameter('settle_time').value
        self.timeout = self.get_parameter('timeout').value
        self.stall_check_time = self.get_parameter('stall_check_time').value
        self.stall_threshold = math.radians(self.get_parameter('stall_threshold').value)
        self.obstacle_distance = self.get_parameter('obstacle_distance').value

        # --- State ---
        self.state = State.IDLE
        self.desired_yaw = None
        self.delta_yaw_needed = None
        self.imu_yaw_at_start = None
        self.current_imu_yaw = None
        self.state_start_time = None
        self.last_succeeded_goal_id = None
        self.kickstart_start_time = None
        self.last_stall_check_yaw = None
        self.last_stall_check_time = None
        self.min_scan_distance = float('inf')

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscriptions ---
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        self.status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.status_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 5)

        # --- Publisher ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Control loop at 20Hz ---
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Rotate After Goal v3 — Kickstart + Safety')
        self.get_logger().info(f'  kickstart: {self.kickstart_speed} rad/s for {self.kickstart_duration}s')
        self.get_logger().info(f'  sustained: {self.rotation_speed} rad/s')
        self.get_logger().info(f'  tolerance: {math.degrees(self.yaw_tolerance):.1f} deg')
        self.get_logger().info(f'  stall:     <{math.degrees(self.stall_threshold):.0f} deg in {self.stall_check_time}s')
        self.get_logger().info(f'  obstacle:  abort if < {self.obstacle_distance}m')
        self.get_logger().info(f'  timeout:   {self.timeout}s')
        self.get_logger().info('=' * 50)

    # ============================================================
    # Callbacks
    # ============================================================

    def goal_callback(self, msg: PoseStamped):
        self.desired_yaw = self.quat_to_yaw(msg.pose.orientation)
        self.state = State.IDLE
        self.get_logger().info(
            f'Stored desired heading: {math.degrees(self.desired_yaw):.1f} deg')

    def imu_callback(self, msg: Imu):
        self.current_imu_yaw = self.quat_to_yaw(msg.orientation)

    def scan_callback(self, msg: LaserScan):
        """Track minimum obstacle distance from LIDAR."""
        try:
            ranges = np.array(msg.ranges)
            valid = ranges[np.isfinite(ranges) & (ranges > msg.range_min)]
            self.min_scan_distance = float(np.min(valid)) if len(valid) > 0 else float('inf')
        except Exception:
            self.min_scan_distance = float('inf')

    def status_callback(self, msg: GoalStatusArray):
        for status in msg.status_list:
            if status.status == 4:
                goal_id = bytes(status.goal_info.goal_id.uuid)
                if (goal_id != self.last_succeeded_goal_id
                        and self.desired_yaw is not None
                        and self.state == State.IDLE):
                    self.last_succeeded_goal_id = goal_id
                    self.transition_to(State.SETTLING)
                    self.get_logger().info(
                        'Nav2 goal succeeded -> settling before rotation')

    # ============================================================
    # Control loop
    # ============================================================

    def control_loop(self):
        now = self.get_clock().now()

        if self.state == State.IDLE or self.state == State.STALLED:
            return

        # --- SETTLING: wait for robot to stop ---
        elif self.state == State.SETTLING:
            elapsed = self.elapsed_seconds(now)
            if elapsed < self.settle_time:
                return

            current_map_yaw = self.get_map_heading()
            if current_map_yaw is None:
                self.get_logger().warn('Cannot read TF -> aborting rotation')
                self.transition_to(State.IDLE)
                return

            self.delta_yaw_needed = self.normalize_angle(
                self.desired_yaw - current_map_yaw)

            self.get_logger().info(
                f'Current: {math.degrees(current_map_yaw):.1f} deg, '
                f'Desired: {math.degrees(self.desired_yaw):.1f} deg, '
                f'Delta: {math.degrees(self.delta_yaw_needed):.1f} deg')

            if abs(self.delta_yaw_needed) < self.yaw_tolerance:
                self.get_logger().info('Already at desired heading')
                self.desired_yaw = None
                self.transition_to(State.IDLE)
                return

            if self.current_imu_yaw is None:
                self.get_logger().warn('No IMU data -> aborting')
                self.transition_to(State.IDLE)
                return

            # Check obstacle safety before starting
            if self.min_scan_distance < self.obstacle_distance:
                self.get_logger().warn(
                    f'Obstacle at {self.min_scan_distance:.2f}m -> aborting rotation')
                self.desired_yaw = None
                self.transition_to(State.IDLE)
                return

            self.imu_yaw_at_start = self.current_imu_yaw
            self.kickstart_start_time = now
            self.last_stall_check_yaw = 0.0
            self.last_stall_check_time = now
            self.transition_to(State.KICKSTART)
            self.get_logger().info(
                f'KICKSTART: {math.degrees(self.delta_yaw_needed):.1f} deg '
                f'at {self.kickstart_speed} rad/s for {self.kickstart_duration}s')

        # --- KICKSTART: high-power pulse to break static friction ---
        elif self.state == State.KICKSTART:
            elapsed_total = self.elapsed_seconds(now)
            kick_elapsed = (now - self.kickstart_start_time).nanoseconds / 1e9

            # Timeout check
            if elapsed_total > self.timeout:
                self.stop_and_idle(f'TIMEOUT after {self.timeout}s')
                return

            # Obstacle check during kickstart
            if self.min_scan_distance < self.obstacle_distance:
                self.stop_and_idle(
                    f'OBSTACLE at {self.min_scan_distance:.2f}m -> abort')
                return

            # Check if already reached target during kickstart
            rotated_so_far = -self.normalize_angle(
                self.current_imu_yaw - self.imu_yaw_at_start)
            remaining = self.normalize_angle(
                self.delta_yaw_needed - rotated_so_far)

            if abs(remaining) < self.yaw_tolerance:
                self.stop_and_idle(
                    f'Target reached during kickstart! '
                    f'Rotated {math.degrees(rotated_so_far):.1f} deg')
                return

            # Send kickstart command
            cmd = Twist()
            if remaining > 0:
                cmd.angular.z = -self.kickstart_speed
            else:
                cmd.angular.z = self.kickstart_speed
            self.cmd_vel_pub.publish(cmd)

            # Transition to sustained after kickstart duration
            if kick_elapsed >= self.kickstart_duration:
                self.last_stall_check_yaw = rotated_so_far
                self.last_stall_check_time = now
                self.transition_to(State.ROTATING)
                self.get_logger().info(
                    f'Kickstart done ({math.degrees(rotated_so_far):.1f} deg) '
                    f'-> sustained at {self.rotation_speed} rad/s')

        # --- ROTATING: sustained speed with stall detection ---
        elif self.state == State.ROTATING:
            elapsed_total = (now - self.kickstart_start_time).nanoseconds / 1e9 + self.kickstart_duration

            # Timeout
            if elapsed_total > self.timeout:
                self.stop_and_idle(f'TIMEOUT after {self.timeout:.0f}s')
                return

            # Obstacle safety
            if self.min_scan_distance < self.obstacle_distance:
                self.stop_and_idle(
                    f'OBSTACLE at {self.min_scan_distance:.2f}m -> abort')
                return

            # IMU tracking (negated for Yahboom convention)
            rotated_so_far = -self.normalize_angle(
                self.current_imu_yaw - self.imu_yaw_at_start)
            remaining = self.normalize_angle(
                self.delta_yaw_needed - rotated_so_far)

            # Target reached?
            if abs(remaining) < self.yaw_tolerance:
                self.stop_and_idle(
                    f'Target reached! '
                    f'Rotated {math.degrees(rotated_so_far):.1f} deg, '
                    f'remaining: {math.degrees(remaining):.1f} deg')
                return

            # Stall detection: check progress every stall_check_time seconds
            stall_elapsed = (now - self.last_stall_check_time).nanoseconds / 1e9
            if stall_elapsed >= self.stall_check_time:
                progress = abs(rotated_so_far - self.last_stall_check_yaw)
                if progress < self.stall_threshold:
                    self.get_logger().error(
                        f'STALL DETECTED: only {math.degrees(progress):.1f} deg '
                        f'in {self.stall_check_time}s (need >{math.degrees(self.stall_threshold):.0f} deg)')
                    self.stop_and_stall('Rotation stalled — wheels slipping on carpet')
                    return
                else:
                    self.get_logger().info(
                        f'  progress check: {math.degrees(progress):.1f} deg in {self.stall_check_time}s — OK')
                self.last_stall_check_yaw = rotated_so_far
                self.last_stall_check_time = now

            # Send sustained rotation command
            cmd = Twist()
            if remaining > 0:
                cmd.angular.z = -self.rotation_speed
            else:
                cmd.angular.z = self.rotation_speed
            self.cmd_vel_pub.publish(cmd)

            # Log every 0.5s
            elapsed_state = self.elapsed_seconds(now)
            if int(elapsed_state * 2) > int((elapsed_state - 0.05) * 2):
                self.get_logger().info(
                    f'  rotating... done={math.degrees(rotated_so_far):.1f} deg, '
                    f'remaining={math.degrees(remaining):.1f} deg, '
                    f'scan={self.min_scan_distance:.2f}m')

    # ============================================================
    # Helpers
    # ============================================================

    def transition_to(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now()

    def stop_and_idle(self, reason: str):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.cmd_vel_pub.publish(stop_cmd)
        self.state = State.IDLE
        self.desired_yaw = None
        self.delta_yaw_needed = None
        self.get_logger().info(f'Rotation complete: {reason}')

    def stop_and_stall(self, reason: str):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.cmd_vel_pub.publish(stop_cmd)
        self.state = State.STALLED
        self.desired_yaw = None
        self.delta_yaw_needed = None
        self.get_logger().error(f'Rotation FAILED: {reason}')

    def elapsed_seconds(self, now) -> float:
        if self.state_start_time is None:
            return 0.0
        return (now - self.state_start_time).nanoseconds / 1e9

    def get_map_heading(self) -> float:
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            return self.quat_to_yaw(transform.transform.rotation)
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    @staticmethod
    def quat_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def destroy_node(self):
        self.cmd_vel_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RotateAfterGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()