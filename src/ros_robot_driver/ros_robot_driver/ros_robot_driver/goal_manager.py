#!/usr/bin/env python3
"""
Goal Manager v3 — Smart Rotate-First Navigation for Mecanum Robot

Architecture:
  1. RViz click → /goal_pose → this node intercepts
  2. Phase A: Rotate CCW to face goal (ONLY if CCW ≤ 180°)
  3. Phase B: Nav2 drives to goal (forward if rotated, strafe if not)
  4. Phase C: Rotate CCW to final heading (ONLY if CCW ≤ 180°)

Smart Rotation Selection:
  CW rotation stalls on carpet (mechanically confirmed).
  CCW always works. BUT rotating 339° CCW to avoid 21° CW is impractical.
  
  Rule: Only rotate if CCW path ≤ 180°.
        If CCW > 180° → skip rotation, let Nav2 strafe.
  
  Result:
    Goal to the left (0° to 180°):  rotate CCW → drive forward
    Goal to the right (0° to -180°): Nav2 strafes directly (proven working)

v3 fixes (from 10-test analysis):
  - rotation_timeout: 30 → 60s (rotation at ~3 deg/s needs 40s for 118°)
  - stall_threshold: 3° → 5° (slow rotation is normal on carpet)
  - 2s settle after Phase A rotation before Phase B (AMCL reconvergence)

Obstacle Safety:
  Subscribes to /scan. Aborts rotation if obstacle < 0.20m.
  Nav2 handles obstacles during driving via costmap (RPLIDAR + RealSense).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, LaserScan
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import rclpy.time
import rclpy.duration
import math
import time as pytime
import numpy as np


class State:
    IDLE = 0
    ROTATE_KICKSTART = 1
    ROTATE_SUSTAIN = 2
    SETTLING = 3          # NEW: wait for AMCL after rotation
    NAVIGATING = 4
    HEADING_KICKSTART = 5
    HEADING_SUSTAIN = 6


class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')

        # --- Parameters ---
        self.declare_parameter('kickstart_speed', 0.65)
        self.declare_parameter('kickstart_duration', 0.5)
        self.declare_parameter('rotation_speed', 0.50)
        self.declare_parameter('yaw_tolerance', 0.15)        # 8.6 deg
        self.declare_parameter('face_tolerance', 0.25)        # 14 deg
        self.declare_parameter('settle_time', 0.5)
        self.declare_parameter('rotation_timeout', 60.0)      # was 30 — 118° needs ~40s
        self.declare_parameter('post_rotation_settle', 2.0)   # NEW: AMCL reconvergence
        self.declare_parameter('stall_check_time', 3.0)
        self.declare_parameter('stall_threshold', 5.0)        # was 3 — slow rotation is normal
        self.declare_parameter('obstacle_distance', 0.20)
        self.declare_parameter('skip_face_threshold', 0.30)   # skip if goal < 0.3m
        self.declare_parameter('max_ccw_rotation', 3.14159)   # 180° — skip if CCW > this

        self.kickstart_speed = self.get_parameter('kickstart_speed').value
        self.kickstart_duration = self.get_parameter('kickstart_duration').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.face_tolerance = self.get_parameter('face_tolerance').value
        self.settle_time = self.get_parameter('settle_time').value
        self.rotation_timeout = self.get_parameter('rotation_timeout').value
        self.post_rotation_settle = self.get_parameter('post_rotation_settle').value
        self.stall_check_time = self.get_parameter('stall_check_time').value
        self.stall_threshold = math.radians(self.get_parameter('stall_threshold').value)
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.skip_face_threshold = self.get_parameter('skip_face_threshold').value
        self.max_ccw_rotation = self.get_parameter('max_ccw_rotation').value

        # --- State ---
        self.state = State.IDLE
        self.goal_pose = None
        self.desired_heading = None
        self.ccw_rotation_needed = 0.0
        self.cumulative_rotation = 0.0
        self.prev_imu_yaw = None
        self.current_imu_yaw = None
        self.state_start_time = None
        self.kickstart_start_time = None
        self.last_stall_check_rotation = 0.0
        self.last_stall_check_time = None
        self.min_scan_distance = float('inf')
        self.nav2_goal_handle = None
        self.is_face_phase = True  # True = Phase A, False = Phase C

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Nav2 Action Client ---
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- Subscriptions ---
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 5)

        # --- Publisher ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Control loop at 20Hz ---
        self.control_timer = self.create_timer(0.05, self.control_loop)

        max_deg = math.degrees(self.max_ccw_rotation)
        self.get_logger().info('=' * 55)
        self.get_logger().info('Goal Manager v3 — Smart Rotate-First')
        self.get_logger().info(f'  kickstart:  {self.kickstart_speed} rad/s for {self.kickstart_duration}s')
        self.get_logger().info(f'  sustained:  {self.rotation_speed} rad/s')
        self.get_logger().info(f'  face_tol:   {math.degrees(self.face_tolerance):.0f} deg')
        self.get_logger().info(f'  heading_tol:{math.degrees(self.yaw_tolerance):.0f} deg')
        self.get_logger().info(f'  max_ccw:    {max_deg:.0f}° (skip if CCW > this)')
        self.get_logger().info(f'  timeout:    {self.rotation_timeout:.0f}s')
        self.get_logger().info(f'  stall:      <{math.degrees(self.stall_threshold):.0f}° in {self.stall_check_time}s')
        self.get_logger().info(f'  settle:     {self.post_rotation_settle}s after rotation')
        self.get_logger().info(f'  obstacle:   abort if < {self.obstacle_distance}m')
        self.get_logger().info('=' * 55)

    # ============================================================
    # Callbacks
    # ============================================================

    def goal_callback(self, msg: PoseStamped):
        if self.state != State.IDLE:
            self.get_logger().warn('Cancelling current goal for new one')
            self.cancel_everything()

        self.goal_pose = msg
        self.desired_heading = self.quat_to_yaw(msg.pose.orientation)

        current_pos = self.get_map_pose()
        if current_pos is None:
            self.get_logger().error('Cannot get robot pose from TF')
            return

        curr_x, curr_y, curr_yaw = current_pos
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        distance = math.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)

        self.get_logger().info(
            f'New goal: ({goal_x:.2f}, {goal_y:.2f}) heading={math.degrees(self.desired_heading):.0f}°, '
            f'distance={distance:.2f}m')

        # Skip face rotation for very close goals
        if distance < self.skip_face_threshold:
            self.get_logger().info(f'Goal is close ({distance:.2f}m) — skip face rotation, Nav2 strafes')
            self.start_navigation()
            return

        # Compute bearing to goal
        bearing = math.atan2(goal_y - curr_y, goal_x - curr_x)
        delta = self.normalize_angle(bearing - curr_yaw)

        self.get_logger().info(
            f'Current heading: {math.degrees(curr_yaw):.0f}°, '
            f'Bearing to goal: {math.degrees(bearing):.0f}°, '
            f'Delta: {math.degrees(delta):.0f}°')

        # Already facing goal?
        if abs(delta) < self.face_tolerance:
            self.get_logger().info('Already facing goal — skip rotation')
            self.start_navigation()
            return

        # Convert to CCW rotation amount
        if delta > 0:
            ccw_amount = delta
        else:
            ccw_amount = delta + 2 * math.pi

        # SMART SELECTION: Skip if CCW > 180°
        if ccw_amount > self.max_ccw_rotation:
            self.get_logger().info(
                f'CCW rotation would be {math.degrees(ccw_amount):.0f}° (>{math.degrees(self.max_ccw_rotation):.0f}°) '
                f'— skip rotation, Nav2 strafes')
            self.start_navigation()
            return

        # Start CCW rotation
        self.start_rotation(ccw_amount, is_face=True)

    def imu_callback(self, msg: Imu):
        new_yaw = self.quat_to_yaw(msg.orientation)

        if self.prev_imu_yaw is not None and self.state in (
                State.ROTATE_KICKSTART, State.ROTATE_SUSTAIN,
                State.HEADING_KICKSTART, State.HEADING_SUSTAIN):
            small_delta = -self.normalize_angle(new_yaw - self.prev_imu_yaw)
            if small_delta > -0.05:
                self.cumulative_rotation += max(0, small_delta)

        self.prev_imu_yaw = new_yaw
        self.current_imu_yaw = new_yaw

    def scan_callback(self, msg: LaserScan):
        try:
            ranges = np.array(msg.ranges)
            valid = ranges[np.isfinite(ranges) & (ranges > msg.range_min)]
            self.min_scan_distance = float(np.min(valid)) if len(valid) > 0 else float('inf')
        except Exception:
            self.min_scan_distance = float('inf')

    # ============================================================
    # Rotation start helper
    # ============================================================

    def start_rotation(self, ccw_amount, is_face=True):
        """Start a CCW rotation of ccw_amount radians."""
        self.is_face_phase = is_face
        self.ccw_rotation_needed = ccw_amount
        phase_name = "PHASE A (face goal)" if is_face else "PHASE C (final heading)"

        if self.min_scan_distance < self.obstacle_distance:
            self.get_logger().warn(
                f'Obstacle at {self.min_scan_distance:.2f}m — cannot rotate, Nav2 strafes')
            if is_face:
                self.start_navigation()
            else:
                self.transition_to(State.IDLE)
                self.log_complete()
            return

        if self.current_imu_yaw is None:
            self.get_logger().error('No IMU data — skipping rotation')
            if is_face:
                self.start_navigation()
            else:
                self.transition_to(State.IDLE)
                self.log_complete()
            return

        self.get_logger().info(
            f'{phase_name}: Rotate CCW {math.degrees(ccw_amount):.0f}°')

        self.cumulative_rotation = 0.0
        self.prev_imu_yaw = self.current_imu_yaw
        self.last_stall_check_rotation = 0.0
        self.kickstart_start_time = self.get_clock().now()
        self.last_stall_check_time = self.get_clock().now()

        if is_face:
            self.transition_to(State.ROTATE_KICKSTART)
        else:
            self.transition_to(State.HEADING_KICKSTART)

    # ============================================================
    # Control loop
    # ============================================================

    def control_loop(self):
        now = self.get_clock().now()

        if self.state == State.IDLE:
            return

        # --- KICKSTART ---
        elif self.state in (State.ROTATE_KICKSTART, State.HEADING_KICKSTART):
            elapsed = (now - self.kickstart_start_time).nanoseconds / 1e9

            if self.min_scan_distance < self.obstacle_distance:
                self.stop_rotation(f'OBSTACLE at {self.min_scan_distance:.2f}m')
                return

            remaining = self.ccw_rotation_needed - self.cumulative_rotation
            if remaining < self.get_current_tolerance():
                self.rotation_complete()
                return

            cmd = Twist()
            cmd.angular.z = -self.kickstart_speed  # CCW
            self.cmd_vel_pub.publish(cmd)

            if elapsed >= self.kickstart_duration:
                next_state = (State.ROTATE_SUSTAIN
                              if self.state == State.ROTATE_KICKSTART
                              else State.HEADING_SUSTAIN)
                self.last_stall_check_rotation = self.cumulative_rotation
                self.last_stall_check_time = now
                self.transition_to(next_state)
                self.get_logger().info(
                    f'Kickstart done ({math.degrees(self.cumulative_rotation):.0f}°) → sustained')

        # --- SUSTAINED ROTATION ---
        elif self.state in (State.ROTATE_SUSTAIN, State.HEADING_SUSTAIN):
            total_elapsed = (now - self.kickstart_start_time).nanoseconds / 1e9

            if total_elapsed > self.rotation_timeout:
                self.stop_rotation(f'TIMEOUT after {self.rotation_timeout:.0f}s')
                return

            if self.min_scan_distance < self.obstacle_distance:
                self.stop_rotation(f'OBSTACLE at {self.min_scan_distance:.2f}m')
                return

            remaining = self.ccw_rotation_needed - self.cumulative_rotation
            tolerance = self.get_current_tolerance()

            if remaining < tolerance:
                self.rotation_complete()
                return

            # Stall detection
            stall_elapsed = (now - self.last_stall_check_time).nanoseconds / 1e9
            if stall_elapsed >= self.stall_check_time:
                progress = self.cumulative_rotation - self.last_stall_check_rotation
                if progress < self.stall_threshold:
                    self.get_logger().error(
                        f'STALL: {math.degrees(progress):.1f}° in {self.stall_check_time}s')
                    self.stop_rotation('Wheels slipping on carpet')
                    return
                self.last_stall_check_rotation = self.cumulative_rotation
                self.last_stall_check_time = now

            cmd = Twist()
            cmd.angular.z = -self.rotation_speed  # CCW
            self.cmd_vel_pub.publish(cmd)

            # Log every 1s
            state_elapsed = self.elapsed_seconds(now)
            if int(state_elapsed) > int(state_elapsed - 0.05):
                phase = "FACE" if self.state == State.ROTATE_SUSTAIN else "HEADING"
                self.get_logger().info(
                    f'  [{phase}] done={math.degrees(self.cumulative_rotation):.0f}° '
                    f'of {math.degrees(self.ccw_rotation_needed):.0f}°, '
                    f'remain={math.degrees(remaining):.0f}°, '
                    f'scan={self.min_scan_distance:.2f}m')

        # --- SETTLING: wait for AMCL after rotation ---
        elif self.state == State.SETTLING:
            elapsed = self.elapsed_seconds(now)
            if elapsed >= self.post_rotation_settle:
                self.get_logger().info('Settle complete → starting Nav2')
                self.start_navigation()

        # --- NAVIGATING ---
        elif self.state == State.NAVIGATING:
            pass

    # ============================================================
    # Phase transitions
    # ============================================================

    def rotation_complete(self):
        self.send_stop()

        if self.is_face_phase:
            self.get_logger().info(
                f'PHASE A complete: rotated CCW '
                f'{math.degrees(self.cumulative_rotation):.0f}° '
                f'→ settling {self.post_rotation_settle}s for AMCL')
            self.transition_to(State.SETTLING)
        else:
            self.get_logger().info(
                f'PHASE C complete: heading aligned '
                f'({math.degrees(self.cumulative_rotation):.0f}° CCW)')
            self.transition_to(State.IDLE)
            self.log_complete()

    def start_navigation(self):
        self.transition_to(State.NAVIGATING)

        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            self.transition_to(State.IDLE)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose

        self.get_logger().info(
            f'PHASE B: Nav2 driving to '
            f'({self.goal_pose.pose.position.x:.2f}, '
            f'{self.goal_pose.pose.position.y:.2f})')

        send_future = self.nav2_client.send_goal_async(
            goal_msg, feedback_callback=self.nav2_feedback)
        send_future.add_done_callback(self.nav2_goal_response)

    def nav2_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected goal')
            self.transition_to(State.IDLE)
            return

        self.nav2_goal_handle = goal_handle
        self.get_logger().info('Nav2 accepted goal — navigating...')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result)

    def nav2_feedback(self, feedback_msg):
        pass

    def nav2_result(self, future):
        result = future.result()
        status = result.status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('PHASE B complete: Nav2 reached goal')
            self.start_heading_alignment()
        elif status == 5:  # CANCELED
            self.get_logger().warn('Nav2 goal was cancelled')
            self.transition_to(State.IDLE)
        else:
            self.get_logger().error(f'Nav2 failed with status {status}')
            self.transition_to(State.IDLE)

    def start_heading_alignment(self):
        if self.desired_heading is None:
            self.transition_to(State.IDLE)
            self.log_complete()
            return

        current_pos = self.get_map_pose()
        if current_pos is None:
            self.get_logger().warn('Cannot get heading for Phase C')
            self.transition_to(State.IDLE)
            self.log_complete()
            return

        _, _, curr_yaw = current_pos
        delta = self.normalize_angle(self.desired_heading - curr_yaw)

        self.get_logger().info(
            f'PHASE C: current={math.degrees(curr_yaw):.0f}°, '
            f'desired={math.degrees(self.desired_heading):.0f}°, '
            f'delta={math.degrees(delta):.0f}°')

        if abs(delta) < self.yaw_tolerance:
            self.get_logger().info('Already at desired heading')
            self.transition_to(State.IDLE)
            self.log_complete()
            return

        # CCW amount
        if delta > 0:
            ccw_amount = delta
        else:
            ccw_amount = delta + 2 * math.pi

        # SMART SELECTION: Skip if CCW > 180°
        if ccw_amount > self.max_ccw_rotation:
            self.get_logger().info(
                f'Heading CCW would be {math.degrees(ccw_amount):.0f}° '
                f'(>{math.degrees(self.max_ccw_rotation):.0f}°) — skipping')
            self.transition_to(State.IDLE)
            self.log_complete()
            return

        self.start_rotation(ccw_amount, is_face=False)

    # ============================================================
    # Helpers
    # ============================================================

    def get_current_tolerance(self):
        if self.is_face_phase:
            return self.face_tolerance
        return self.yaw_tolerance

    def log_complete(self):
        self.get_logger().info('=' * 40)
        self.get_logger().info('GOAL COMPLETE')
        self.get_logger().info('=' * 40)

    def transition_to(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now()

    def send_stop(self):
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
        self.cmd_vel_pub.publish(stop)

    def stop_rotation(self, reason):
        self.send_stop()
        self.get_logger().error(f'Rotation FAILED: {reason}')
        # On failure, try Nav2 strafing instead if this was face phase
        if self.is_face_phase:
            self.get_logger().info('Falling back to Nav2 strafing')
            self.start_navigation()
        else:
            self.transition_to(State.IDLE)

    def cancel_everything(self):
        self.send_stop()
        if self.nav2_goal_handle is not None:
            try:
                self.nav2_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.nav2_goal_handle = None
        self.transition_to(State.IDLE)

    def elapsed_seconds(self, now):
        if self.state_start_time is None:
            return 0.0
        return (now - self.state_start_time).nanoseconds / 1e9

    def get_map_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = self.quat_to_yaw(t.transform.rotation)
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    @staticmethod
    def quat_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def destroy_node(self):
        try:
            self.cancel_everything()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()