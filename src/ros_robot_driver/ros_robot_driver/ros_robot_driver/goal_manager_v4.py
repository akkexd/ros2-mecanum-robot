#!/usr/bin/env python3
"""
goal_manager_v4.py — Autonomous Multi-Room Navigation with Doorway Transit

Changes in this version:
  1. HEADING CORRECTION: Robot always rotates to face next waypoint before driving
     - CW rotation now works for open nav segments (not just doorway align)
     - Eliminates backward/sideways driving through hallway
  2. REVERSE DOORWAY: Skip doorway transit for reverse direction
     - Nav2 handles reverse doorway crossing directly (requires inflation_radius: 0.20)
     - Fixes the hang at DT Step 2 during kitchen->bedroom route
"""
import math, time, yaml, os
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu
from action_msgs.msg import GoalStatus

CCW_SPEED = 1.0 #0.65
KICKSTART_SPEED = 1.3 #0.85
KICKSTART_DURATION = 0.5
ROTATION_TOLERANCE = 0.15
STALL_THRESHOLD = math.radians(5)
STALL_CHECK_INTERVAL = 3.0
ROTATION_TIMEOUT = 60.0
SETTLE_TIME = 2.0

TRANSIT_SPEED = 0.03
TRANSIT_TIMEOUT = 30.0
ARRIVAL_TOLERANCE = 0.45
OBSTACLE_STOP_DIST = 0.15
OBSTACLE_SCAN_ANGLE = 30
NAV2_TIMEOUT = 90.0
POSE_TOPIC = '/amcl_pose'

def normalize_angle(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def angle_diff(t, c): return normalize_angle(t - c)
def euclidean_dist(p1, p2): return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def yaw_from_quaternion(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))

def heading_to_point(f, t): return math.atan2(t[1]-f[1], t[0]-f[0])


class DoorwayConfig:
    def __init__(self, p):
        with open(p,'r') as f: data=yaml.safe_load(f)
        self.doorways={}
        for n,d in data.get('doorways',{}).items():
            self.doorways[n]={'center':tuple(d['doorway_center']),'radius':d['zone_radius'],'heading':d['transit_heading'],'staging':tuple(d['staging_point']),'exit':tuple(d['exit_point']),'width':d.get('width_m',0.76)}
        self.waypoints={}
        for n,c in data.get('waypoints',{}).items(): self.waypoints[n]=tuple(c)
        self.edges=[]; self.adjacency={}
        for e in data.get('edges',[]):
            fr,to,dw=e['from'],e['to'],e.get('doorway',None)
            self.edges.append((fr,to,dw))
            self.adjacency.setdefault(fr,[]).append((to,dw))
            self.adjacency.setdefault(to,[]).append((fr,dw))

    def find_nearest_waypoint(self, xy):
        best_n,best_d=None,float('inf')
        for n,c in self.waypoints.items():
            d=euclidean_dist(xy,c)
            if d<best_d: best_d=d; best_n=n
        return best_n,best_d

    def find_path(self, s, g):
        if s==g: return [(g,None)]
        vis={s}; q=deque([(s,[(s,None)])])
        while q:
            cur,path=q.popleft()
            for nb,dw in self.adjacency.get(cur,[]):
                if nb not in vis:
                    vis.add(nb); np=path+[(nb,dw)]
                    if nb==g: return np
                    q.append((nb,np))
        return None


class GoalManagerV4(Node):
    IDLE='IDLE'; PLANNING='PLANNING'; OPEN_NAV='OPEN_NAV'; DOORWAY_TRANSIT='DOORWAY_TRANSIT'
    PHASE_A='PHASE_A'; PHASE_B='PHASE_B'; PHASE_C='PHASE_C'
    DT_STAGE='DT_STAGE'; DT_ALIGN='DT_ALIGN'; DT_DRIVE='DT_DRIVE'

    def __init__(self):
        super().__init__('goal_manager_v4')
        self.declare_parameter('doorway_config','')
        cp=self.get_parameter('doorway_config').get_parameter_value().string_value
        if not cp or not os.path.exists(cp): cp=os.path.expanduser('~/ros2_ws/src/ros_robot_bringup/config/doorway_config.yaml')
        if os.path.exists(cp):
            self.config=DoorwayConfig(cp)
            self.get_logger().info(f'Loaded doorway config: {cp}')
            self.get_logger().info(f'  Doorways: {list(self.config.doorways.keys())}')
            self.get_logger().info(f'  Waypoints: {list(self.config.waypoints.keys())}')
        else: self.config=None; self.get_logger().warn(f'No doorway config at {cp}')

        self.robot_x=0.0; self.robot_y=0.0; self.robot_yaw=0.0; self.pose_received=False
        self.min_scan_distance=float('inf')
        self.state=self.IDLE; self.sub_state=None; self.waypoint_queue=[]
        self.current_target=None; self.current_heading_target=None; self.nav2_goal_active=False
        self.rotation_start_time=None; self.last_stall_check_time=None; self.kickstart_done=False
        self.cumulative_rotation=0.0; self._last_stall_cumulative=0.0; self._needed_rotation=0.0
        self._dt_align_cw=False
        self._phase_a_cw=False
        self.transit_start_time=None; self.active_doorway=None
        self.prev_imu_yaw=None; self.current_imu_yaw=None

        self.cmd_vel_pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.nav2_goal_pub=self.create_publisher(PoseStamped,'/goal_pose_unused',10)
        aq=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL,depth=1)
        self.create_subscription(PoseWithCovarianceStamped,POSE_TOPIC,self.pose_callback,aq)
        self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.create_subscription(Imu,'/imu/data_raw',self.imu_callback,10)
        self.create_subscription(PoseStamped,'/goal_manager/goal',self.goal_callback,10)
        self.create_subscription(PoseStamped,'/goal_pose',self.goal_callback,10)
        self.create_subscription(String,'/goal_manager/named_goal',self.named_goal_callback,10)
        self.timer=self.create_timer(0.1,self.control_loop)
        self.get_logger().info('Goal Manager v4 initialized')

    def pose_callback(self,m):
        self.robot_x=m.pose.pose.position.x; self.robot_y=m.pose.pose.position.y
        self.robot_yaw=yaw_from_quaternion(m.pose.pose.orientation); self.pose_received=True

    def scan_callback(self,m):
        if not m.ranges: return
        n=len(m.ranges); ha=math.radians(OBSTACLE_SCAN_ANGLE); ei=int(ha/m.angle_increment); ws=n-ei
        fr=[]
        for i in list(range(0,ei))+list(range(ws,n)):
            r=m.ranges[i]
            if m.range_min<r<m.range_max: fr.append(r)
        self.min_scan_distance=min(fr) if fr else float('inf')

    def imu_callback(self,m):
        ny=yaw_from_quaternion(m.orientation)
        if self.prev_imu_yaw is not None and self.state in (self.OPEN_NAV,self.DOORWAY_TRANSIT):
            d=-normalize_angle(ny-self.prev_imu_yaw)
            if self._dt_align_cw or self._phase_a_cw:
                if d<0.05: self.cumulative_rotation+=max(0,-d)
            else:
                if d>-0.05: self.cumulative_rotation+=max(0,d)
        self.prev_imu_yaw=ny; self.current_imu_yaw=ny

    def goal_callback(self,m):
        if not self.pose_received: return
        if self.state!=self.IDLE: return
        self.get_logger().info(f'Goal received: ({m.pose.position.x:.2f}, {m.pose.position.y:.2f})')
        self.plan_navigation(m.pose.position.x,m.pose.position.y)

    def named_goal_callback(self,m):
        if not self.pose_received: return
        if not self.config: return
        w=m.data.strip()
        if w not in self.config.waypoints: self.get_logger().error(f'Unknown: {w}'); return
        g=self.config.waypoints[w]
        self.get_logger().info(f'Named goal: {w} -> ({g[0]:.2f},{g[1]:.2f})')
        self.plan_navigation(g[0],g[1])

    def plan_navigation(self,gx,gy):
        self.stop_robot(); self.state=self.PLANNING
        rxy=(self.robot_x,self.robot_y); gxy=(gx,gy)
        if not self.config:
            self.waypoint_queue=[('__direct__',None)]; self.current_target=gxy; self.start_open_nav(gxy); return
        sw,sd=self.config.find_nearest_waypoint(rxy); gw,gd=self.config.find_nearest_waypoint(gxy)
        self.get_logger().info(f'Planning: robot near {sw} ({sd:.2f}m), goal near {gw} ({gd:.2f}m)')
        if sw==gw or (sd<1.0 and gd<1.0 and euclidean_dist(rxy,gxy)<3.0):
            self.get_logger().info('Same-room — direct nav'); self.waypoint_queue=[]; self.start_open_nav(gxy); return
        path=self.config.find_path(sw,gw)
        if not path: self.get_logger().error(f'No path {sw}->{gw}'); self.state=self.IDLE; return
        self.waypoint_queue=path[1:]
        plan_str = " -> ".join(w + ("[" + d + "]" if d else "") for w, d in path)
        self.get_logger().info(f'Plan: {plan_str}')
        if gd>0.3: self.waypoint_queue.append(('__final__',None)); self._final_goal=gxy
        self.execute_next_segment()

    def execute_next_segment(self):
        if not self.waypoint_queue:
            self.get_logger().info('=== NAVIGATION COMPLETE ==='); self.state=self.IDLE; return
        wn,dn=self.waypoint_queue.pop(0)
        if wn=='__final__': txy=self._final_goal
        elif wn=='__direct__': txy=self.current_target
        else: txy=self.config.waypoints[wn]
        self.get_logger().info(f'Segment: -> {wn} ({txy[0]:.2f},{txy[1]:.2f}){" via "+dn if dn else ""}')
        if dn: self.start_doorway_transit(dn,txy)
        else: self.start_open_nav(txy)

    # =========================================================================
    # OPEN NAV — ALWAYS rotates to face target (CW or CCW)
    # =========================================================================
    def start_open_nav(self,txy):
        self.state=self.OPEN_NAV; self.current_target=txy
        self._dt_align_cw=False; self._phase_a_cw=False
        rxy=(self.robot_x,self.robot_y)
        dh=heading_to_point(rxy,txy)
        diff=angle_diff(dh,self.robot_yaw)

        if abs(diff)<ROTATION_TOLERANCE:
            self.get_logger().info('Already facing — skip Phase A')
            self.start_phase_b()
            return

        if diff<0:
            cw_amount=abs(diff)
            if cw_amount<math.radians(20):
                self.get_logger().info(f'Small angle ({math.degrees(cw_amount):.1f} deg) — skip Phase A')
                self.start_phase_b()
            elif cw_amount<math.radians(65):
                self.get_logger().info(f'Phase A: CW {math.degrees(cw_amount):.1f} deg')
                self._phase_a_cw=True
                self._start_cw_rotation(dh, cw_amount)
            else:
                ccw_amount=2*math.pi-cw_amount
                self.get_logger().info(f'Phase A: CCW {math.degrees(ccw_amount):.1f} deg (long way)')
                self._phase_a_cw=False
                self.start_rotation(dh)
        else:
            self.get_logger().info(f'Phase A: CCW {math.degrees(diff):.1f} deg')
            self._phase_a_cw=False
            self.start_rotation(dh)

    def _start_cw_rotation(self, target_heading, amount):
        self.sub_state=self.PHASE_A
        self.current_heading_target=target_heading
        self.rotation_start_time=time.time()
        self.last_stall_check_time=time.time()
        self.cumulative_rotation=0.0
        self._last_stall_cumulative=0.0
        self._needed_rotation=amount
        self.prev_imu_yaw=self.current_imu_yaw
        self.kickstart_done=False

    def start_rotation(self,th):
        self.sub_state=self.PHASE_A; self.current_heading_target=th
        self.rotation_start_time=time.time(); self.last_stall_check_time=time.time()
        self.cumulative_rotation=0.0; self._last_stall_cumulative=0.0
        self.prev_imu_yaw=self.current_imu_yaw; self.kickstart_done=False
        self._needed_rotation=abs(angle_diff(th,self.robot_yaw))

    def start_phase_b(self):
        self.sub_state=self.PHASE_B; self.nav2_goal_active=True
        self._phase_a_cw=False; self._dt_align_cw=False
        time.sleep(2.0)
        gm=PoseStamped(); gm.header.frame_id='map'; gm.header.stamp=self.get_clock().now().to_msg()
        gm.pose.position.x=self.current_target[0]; gm.pose.position.y=self.current_target[1]
        h=heading_to_point((self.robot_x,self.robot_y),self.current_target)
        gm.pose.orientation.z=math.sin(h/2); gm.pose.orientation.w=math.cos(h/2)
        self.nav2_goal_pub.publish(gm)
        self.get_logger().info(f'Phase B: Nav2 goal -> ({self.current_target[0]:.2f},{self.current_target[1]:.2f})')
        self.phase_b_start_time=time.time()

    def handle_open_nav(self):
        if self.sub_state==self.PHASE_A:
            if self._phase_a_cw:
                self._handle_cw_rotation_phase_a()
            else:
                self.handle_rotation()
        elif self.sub_state==self.PHASE_B: self.handle_phase_b()
        elif self.sub_state==self.PHASE_C: self.handle_rotation()

    def _handle_cw_rotation_phase_a(self):
        diff=angle_diff(self.current_heading_target,self.robot_yaw)
        el=time.time()-self.rotation_start_time

        if abs(diff)<ROTATION_TOLERANCE or el>ROTATION_TIMEOUT:
            self.stop_robot(); time.sleep(SETTLE_TIME)
            self.get_logger().info(f'Phase A CW complete (err={math.degrees(diff):.1f} deg)')
            self._phase_a_cw=False; self.start_phase_b()
            return

        if self.cumulative_rotation>=(self._needed_rotation-0.10):
            self.stop_robot(); time.sleep(SETTLE_TIME)
            self.get_logger().info(f'Phase A CW via IMU ({math.degrees(self.cumulative_rotation):.0f} of {math.degrees(self._needed_rotation):.0f} deg)')
            self._phase_a_cw=False; self.start_phase_b()
            return

        now=time.time()
        if now-self.last_stall_check_time>STALL_CHECK_INTERVAL:
            ip=self.cumulative_rotation-self._last_stall_cumulative
            if ip<STALL_THRESHOLD:
                self.get_logger().warn('Phase A CW stalled — proceed to Phase B')
                self.stop_robot(); time.sleep(SETTLE_TIME)
                self._phase_a_cw=False; self.start_phase_b()
                return
            else:
                self.get_logger().info(f'Phase A CW: IMU={math.degrees(self.cumulative_rotation):.0f} deg, err={math.degrees(diff):.0f} deg')
            self.last_stall_check_time=now; self._last_stall_cumulative=self.cumulative_rotation

        if self.min_scan_distance<OBSTACLE_STOP_DIST:
            self.get_logger().warn(f'Obstacle {self.min_scan_distance:.2f}m — pause')
            self.stop_robot(); return

        cmd=Twist()
        if not self.kickstart_done:
            cmd.angular.z=KICKSTART_SPEED
            if el>KICKSTART_DURATION: self.kickstart_done=True
        else:
            cmd.angular.z=CCW_SPEED
        self.cmd_vel_pub.publish(cmd)

    def handle_rotation(self):
        el=time.time()-self.rotation_start_time
        if el>ROTATION_TIMEOUT:
            self.get_logger().warn('Rotation timeout'); self.stop_robot(); time.sleep(SETTLE_TIME)
            if self.sub_state==self.PHASE_A: self.start_phase_b()
            else: self.rotation_complete()
            return

        diff=angle_diff(self.current_heading_target,self.robot_yaw)
        if abs(diff)<ROTATION_TOLERANCE:
            self.get_logger().info(f'Rotation complete (err={math.degrees(diff):.1f} deg, IMU={math.degrees(self.cumulative_rotation):.0f} deg)')
            self.stop_robot(); time.sleep(SETTLE_TIME)
            if self.sub_state==self.PHASE_A: self.start_phase_b()
            else: self.rotation_complete()
            return

        if self.cumulative_rotation>=(self._needed_rotation-0.3):
            self.get_logger().info(f'Rotation complete via IMU ({math.degrees(self.cumulative_rotation):.0f} of {math.degrees(self._needed_rotation):.0f} deg)')
            self.stop_robot(); time.sleep(SETTLE_TIME)
            if self.sub_state==self.PHASE_A: self.start_phase_b()
            else: self.rotation_complete()
            return

        now=time.time()
        if now-self.last_stall_check_time>STALL_CHECK_INTERVAL:
            ip=self.cumulative_rotation-self._last_stall_cumulative
            if ip<STALL_THRESHOLD:
                self.get_logger().warn(f'Stalled ({math.degrees(ip):.1f} deg) — kickstart')
                self.kickstart_done=False
            else:
                self.get_logger().info(f'Rotating... IMU={math.degrees(self.cumulative_rotation):.0f} deg, err={math.degrees(diff):.0f} deg')
            self.last_stall_check_time=now; self._last_stall_cumulative=self.cumulative_rotation

        if self.min_scan_distance<OBSTACLE_STOP_DIST:
            self.get_logger().warn(f'Obstacle {self.min_scan_distance:.2f}m — pause')
            self.stop_robot(); return

        cmd=Twist()
        if not self.kickstart_done:
            cmd.angular.z=-KICKSTART_SPEED
            if el>KICKSTART_DURATION: self.kickstart_done=True
        else: cmd.angular.z=-CCW_SPEED
        self.cmd_vel_pub.publish(cmd)

    def handle_phase_b(self):
        dist=euclidean_dist((self.robot_x,self.robot_y),self.current_target)
        el=time.time()-self.phase_b_start_time
        if dist<ARRIVAL_TOLERANCE:
            self.get_logger().info(f'Phase B: arrived (dist={dist:.2f}m)')
            self.stop_robot(); self.nav2_goal_active=False
            self.on_segment_complete(); return
        if el>NAV2_TIMEOUT:
            self.get_logger().warn(f'Phase B: timeout {el:.0f}s (dist={dist:.2f}m)')
            self.stop_robot(); self.nav2_goal_active=False
            self.on_segment_complete(); return
        if int(el)%5==0 and el>0:
            self.get_logger().info(f'Phase B: dist={dist:.2f}m, elapsed={el:.0f}s')

    def on_segment_complete(self):
        self.execute_next_segment()

    # =========================================================================
    # DOORWAY TRANSIT
    # =========================================================================
    def start_doorway_transit(self,dn,pdt):
        dw=self.config.doorways[dn]; rxy=(self.robot_x,self.robot_y)
        ds=euclidean_dist(rxy,dw['staging']); de=euclidean_dist(rxy,dw['exit'])

        # Reverse direction: skip doorway transit, Nav2 drives through directly
        if ds>de:
            self.get_logger().info(f'Doorway {dn}: reverse — Nav2 drives through (no transit)')
            self.start_open_nav(pdt)
            return

        # Forward direction: full doorway transit
        self.state=self.DOORWAY_TRANSIT; self.active_doorway=dn
        self._post_doorway_target=pdt
        self._dt_align_cw=False; self._phase_a_cw=False
        self._dt_staging=dw['staging']; self._dt_exit=dw['exit']; self._dt_heading=dw['heading']
        self.get_logger().info(f'Doorway {dn}: forward (heading={math.degrees(dw["heading"]):.0f} deg)')

        self.get_logger().info(f'DT Step 1 (STAGE): -> ({self._dt_staging[0]:.2f},{self._dt_staging[1]:.2f})')
        self.sub_state=self.DT_STAGE; self.current_target=self._dt_staging

        if euclidean_dist(rxy,self._dt_staging)<0.6:
            self.get_logger().info(f'Already near staging ({euclidean_dist(rxy,self._dt_staging):.2f}m) — skip to ALIGN')
            self._start_dt_align(); return

        self._dt_inner_state='phase_a'
        dh=heading_to_point(rxy,self._dt_staging); diff=angle_diff(dh,self.robot_yaw)
        if abs(diff)<ROTATION_TOLERANCE:
            self._dt_inner_state='phase_b'; self.start_phase_b()
        else: self.start_rotation(dh)

    def handle_doorway_transit(self):
        if self.sub_state==self.DT_STAGE: self._handle_dt_stage()
        elif self.sub_state==self.DT_ALIGN: self._handle_dt_align()
        elif self.sub_state==self.DT_DRIVE: self._handle_dt_drive()

    def _handle_dt_stage(self):
        if self._dt_inner_state=='phase_a':
            diff=angle_diff(self.current_heading_target,self.robot_yaw); el=time.time()-self.rotation_start_time
            if abs(diff)<ROTATION_TOLERANCE or el>ROTATION_TIMEOUT:
                self.stop_robot(); time.sleep(SETTLE_TIME)
                self._dt_inner_state='phase_b'; self.start_phase_b(); return
            self.handle_rotation()
        elif self._dt_inner_state=='phase_b':
            dist=euclidean_dist((self.robot_x,self.robot_y),self._dt_staging)
            el=time.time()-self.phase_b_start_time
            if dist<0.45 or el>NAV2_TIMEOUT:
                self.stop_robot(); self.nav2_goal_active=False
                self.get_logger().info(f'DT Step 1 complete (dist={dist:.2f}m)')
                self._start_dt_align(); return
            if int(el)%5==0 and el>0:
                self.get_logger().info(f'DT Stage: dist={dist:.2f}m, elapsed={el:.0f}s')

    def _start_dt_align(self):
        self.sub_state=self.DT_ALIGN; self._phase_a_cw=False
        diff=angle_diff(self._dt_heading,self.robot_yaw)
        self.get_logger().info(f'DT Step 2 (ALIGN): rotate {math.degrees(diff):.1f} deg to heading {math.degrees(self._dt_heading):.0f} deg')

        if abs(diff)<ROTATION_TOLERANCE:
            self.get_logger().info('Already aligned — skip to drive')
            self._dt_align_cw=False; self._start_dt_drive(); return

        if diff<0 and abs(diff)<math.radians(100):
            self.get_logger().info(f'Small CW rotation {math.degrees(diff):.0f} deg for doorway align')
            self._dt_align_cw=True
            self.current_heading_target=self._dt_heading
            self.rotation_start_time=time.time(); self.last_stall_check_time=time.time()
            self.cumulative_rotation=0.0; self._last_stall_cumulative=0.0
            self._needed_rotation=abs(diff); self.prev_imu_yaw=self.current_imu_yaw
            self.kickstart_done=False
            return

        self._dt_align_cw=False
        if diff<0:
            ccw=diff+2*math.pi
            if ccw>math.pi*1.5:
                self.get_logger().warn(f'Large CCW needed ({math.degrees(ccw):.0f} deg) — skip')
                self._start_dt_drive(); return

        self.start_rotation(self._dt_heading)

    def _handle_dt_align(self):
        diff=angle_diff(self._dt_heading,self.robot_yaw); el=time.time()-self.rotation_start_time

        if abs(diff)<ROTATION_TOLERANCE or el>ROTATION_TIMEOUT:
            self.stop_robot(); time.sleep(SETTLE_TIME)
            self.get_logger().info(f'DT Step 2 complete (err={math.degrees(diff):.1f} deg)')
            self._dt_align_cw=False; self._start_dt_drive(); return

        if self.cumulative_rotation>=(self._needed_rotation-0.10):
            self.stop_robot(); time.sleep(SETTLE_TIME)
            self.get_logger().info(f'DT align via IMU ({math.degrees(self.cumulative_rotation):.0f} of {math.degrees(self._needed_rotation):.0f} deg)')
            self._dt_align_cw=False; self._start_dt_drive(); return

        if self._dt_align_cw:
            now=time.time()
            if now-self.last_stall_check_time>STALL_CHECK_INTERVAL:
                ip=self.cumulative_rotation-self._last_stall_cumulative
                if ip<STALL_THRESHOLD:
                    self.get_logger().warn('CW stalled — proceed')
                    self.stop_robot(); time.sleep(SETTLE_TIME)
                    self._dt_align_cw=False; self._start_dt_drive(); return
                else:
                    self.get_logger().info(f'CW aligning... IMU={math.degrees(self.cumulative_rotation):.0f} deg, err={math.degrees(diff):.0f} deg')
                self.last_stall_check_time=now; self._last_stall_cumulative=self.cumulative_rotation

            cmd=Twist()
            if not self.kickstart_done:
                cmd.angular.z=KICKSTART_SPEED
                if el>KICKSTART_DURATION: self.kickstart_done=True
            else: cmd.angular.z=CCW_SPEED
            self.cmd_vel_pub.publish(cmd); return

        self.handle_rotation()

    def _start_dt_drive(self):
        self.sub_state=self.DT_DRIVE
        self._dt_align_cw=False; self._phase_a_cw=False
        self.transit_start_time=time.time()
        dist=euclidean_dist((self.robot_x,self.robot_y),self._dt_exit)
        est=dist/TRANSIT_SPEED if TRANSIT_SPEED>0 else 999
        self.get_logger().info(f'DT Step 3 (DRIVE): cmd_vel {TRANSIT_SPEED} m/s -> exit ({self._dt_exit[0]:.2f},{self._dt_exit[1]:.2f}), dist={dist:.2f}m, est={est:.0f}s')
        self.get_logger().info('*** BYPASSING Nav2 ***')

    def _handle_dt_drive(self):
        rxy=(self.robot_x,self.robot_y)
        dte=euclidean_dist(rxy,self._dt_exit)
        el=time.time()-self.transit_start_time

        if dte<ARRIVAL_TOLERANCE:
            self.stop_robot()
            self.get_logger().info(f'DT Step 3 complete! (dist={dte:.2f}m, time={el:.1f}s)')
            self.get_logger().info(f'=== Doorway [{self.active_doorway}] traversed ===')
            self.active_doorway=None; self.on_segment_complete(); return

        if el>TRANSIT_TIMEOUT:
            self.stop_robot()
            self.get_logger().warn(f'DT drive timeout ({el:.0f}s) dist={dte:.2f}m')
            self.active_doorway=None; self.on_segment_complete(); return

        if self.min_scan_distance<0.08:
            self.stop_robot()
            self.get_logger().warn(f'DT drive: obstacle {self.min_scan_distance:.2f}m — TOO CLOSE'); return

        cmd=Twist()
        he=heading_to_point(rxy,self._dt_exit)
        herr=angle_diff(he,self.robot_yaw)
        cmd.linear.x=TRANSIT_SPEED*math.cos(herr)
        cmd.linear.y=TRANSIT_SPEED*math.sin(herr)
        cmd.angular.z=0.0
        self.cmd_vel_pub.publish(cmd)
        if int(el*2)%4==0:
            self.get_logger().info(f'DT drive: dist={dte:.2f}m, herr={math.degrees(herr):.1f} deg, vx={cmd.linear.x:.3f}, vy={cmd.linear.y:.3f}')

    def rotation_complete(self): self.on_segment_complete()

    def control_loop(self):
        if not self.pose_received: return
        if self.state==self.IDLE: return
        elif self.state==self.PLANNING: return
        elif self.state==self.OPEN_NAV: self.handle_open_nav()
        elif self.state==self.DOORWAY_TRANSIT: self.handle_doorway_transit()

    def stop_robot(self): self.cmd_vel_pub.publish(Twist())
    def destroy_node(self): self.stop_robot(); super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node=GoalManagerV4()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info('Shutting down...')
    finally: node.stop_robot(); node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()