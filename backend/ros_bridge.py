"""ROS 2 adapter. All navigation goes through Nav2 actions, never pixel clicks."""
import asyncio
import io
import math
import os
import threading
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from sensor_msgs.msg import LaserScan, Imu, BatteryState, Image as RosImage
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from nav2_msgs.action import NavigateThroughPoses, ComputePathThroughPoses
from nav2_msgs.msg import SpeedLimit
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Empty
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from rcl_interfaces.srv import SetParameters
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from cv_bridge import CvBridge
import cv2
from core import (ConsoleError, automatic_goals, bounds, grid_likelihood, idle_refine_decision, lap_number, missed_waypoint,
                  aruco_consistent, aruco_jump_ok, marker_pose_from_robot, plan_leads_away, relocalize_decision,
                  robot_pose_from_marker, stalled_here, mission_stop_reason, circle_footprint, footprint_text,
                    map_image, plan_batch, points_within, prune_reached_goals, record_step, remaining_route_distance, start_conflict,
                    validate_waypoints, waypoint_index,
                    world_to_cell, snap_pose as core_snap_pose)


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def position(p):
    return {'x':p.position.x,'y':p.position.y,'yaw':yaw(p.orientation)}


async def ros_future(future, timeout=12):
    start=time.monotonic()
    while not future.done():
        if time.monotonic()-start > timeout:
            raise ConsoleError('ROS_TIMEOUT','ROS 请求超时',504)
        await asyncio.sleep(.025)
    return future.result()


class RosBridge(Node):
    def __init__(self, config):
        rclpy.init()
        super().__init__('mumai_console_bridge')
        self.config=config; self.lock=threading.RLock()
        self.mode='idle';self.started_at=time.time();self.map_meta=None;self.grid=None;self.map_png=None
        self.map_revision=0;self.map_at=0;self.last_scan=None;self.scan_at=0;self.scan_times=deque(maxlen=80)
        self.odom=None;self.odom_at=0;self.imu=None;self.imu_at=0;self.imu_history=deque(maxlen=90)
        self.pose=None;self.pose_at=0;self.match_at=0;self.scan_points=[];self.plan=[];self.amcl=None;self.amcl_at=0;self.match=None
        self.camera_jpeg=None;self.camera_at=0;self.camera_times=deque(maxlen=60);self.battery_voltage=None;self.voltage_at=0;self.bms=None;self.bms_at=0;self.charging=None;self.charging_at=0;self.charging_current=None;self.current_at=0;self.raw_odom=None;self.raw_odom_at=0;self.commanded=None;self.commanded_at=0;self.camera_size=None
        self.error=None;self.epoch=0;self.goal_handle=None;self.pending_goal=None;self.cancel_lock=asyncio.Lock();self.mission={'state':'idle','index':0,'cycle':0,'points':[],'mode':'multi','distance_remaining':None}
        self.speed=config['default_speed_mps'];self.cv=CvBridge();self._camera_encode_at=0
        # 连续巡航：每批下发的目标点数（0=mult 全部剩余 / loop 一圈减一段）
        self.lookahead=int(config.get('cruise_lookahead',4) or 0)
        self.arrival_radius=float(config.get('cruise_arrival_radius',.20))
        self.config_clearance=float(config.get('cruise_clearance',.22))
        self.footprint_center_cache=(.089,0.)      # 车体几何中心，加载地图时由 app 用真实足迹更新
        # 密集航点（录制每 10 cm 一个点）：窗口按距离补足，避免每几个点就停一次
        self.lookahead_m=float(config.get('cruise_lookahead_m',2.5) or 0.)
        self.lookahead_max=int(config.get('cruise_lookahead_max_points',40) or 0)
        self.skip_missed=bool(config.get('skip_missed_points',False))
        self._skipping=False;self._last_skip_at=0.;self._target_distance={}
        # “兜圈就跳过”：越过、停在点旁不动、或规划一开头就朝反方向走，都直接去下一个点
        self.skip_near=float(config.get('cruise_skip_near_m',.25))
        self.skip_stall=float(config.get('cruise_skip_stall_s',5.))
        self.skip_detour=float(config.get('cruise_skip_detour_deg',120.))
        self._stall_ref=None;self._stall_since=None
        # 任务运行中的“车没动”监控：既用于前端提示，也用于卡住时自动跳点
        self.stuck_s=0.;self._move_ref=None;self._move_at=None;self.stuck_skips=0
        self.trace=deque(maxlen=30)                  # 批次/跳过/结束事件，便于事后定位
        self.stuck_after=float(config.get('cruise_stuck_s',8.))
        self.stuck_max=int(config.get('cruise_stuck_max',6))
        # 相机（ArUco 标签）绝对定位校正：看到已知位置的标签就把车拽回绝对位姿
        aruco=dict(config.get('aruco') or {})
        self.aruco={'enabled':bool(aruco.get('enabled',True)),'marker_id':aruco.get('marker_id',582),
                    'frame':aruco.get('frame','console_marker'),'max_jump':float(aruco.get('max_jump_m',.8)),
                    'confirm_frames':int(aruco.get('confirm_frames',3)),'marker_pose':aruco.get('marker_pose')}
        self.aruco_state={'enabled':self.aruco['enabled'],'configured':bool(self.aruco['marker_pose']),'seeing':False,
                          'count':0,'reason':'disabled' if not self.aruco['enabled'] else ('unconfigured' if not self.aruco['marker_pose'] else 'waiting'),
                          'marker_id':self.aruco['marker_id'],'shift_m':None,'shift_deg':None,'at':None}
        self._aruco_seen=None;self._aruco_streak=0;self._aruco_last_at=0.
        # 航行中定位守护：吻合度持续偏低时小窗口吸附重锚，连续失败则安全停车
        self.relocalize_enabled=bool(config.get('cruise_relocalize',True))
        self.relocalize_match=float(config.get('cruise_relocalize_match',.55))
        self.relocalize_after=float(config.get('cruise_relocalize_after_s',2.))
        self.relocalize_interval=float(config.get('cruise_relocalize_interval_s',5.))
        self.relocalize_max=int(config.get('cruise_relocalize_max',4))
        self.relocalize_state={'enabled':self.relocalize_enabled,'count':0,'reason':'disabled','match':None,
                               'shift_m':None,'shift_deg':None,'at':None}
        self._match_low_since=None;self._last_relocalize_at=0.
        self.clearance=float(config.get('cruise_clearance',.22))
        self._like=None;self._like_rev=None
        # 航迹记录（遥控教学）：每 record_step_m 米记一个点，直接可存成路线
        self.record_step_m=float(config.get('record_step_m',.2))
        self.record_max=int(config.get('record_max_points',800))
        self.record={'active':False,'count':0,'distance_m':0.,'points':[],'started_at':None}
        # 静止自动校准：定位准了以后底盘静止时仍会缓慢漂移，周期性做一次小窗口吸附
        self.refine_enabled=bool(config.get('refine_idle',True))
        self.refine_delay=float(config.get('refine_idle_delay_s',6.))
        self.refine_interval=float(config.get('refine_idle_interval_s',8.))
        self.refine_shift=float(config.get('refine_idle_shift_m',.18))
        self.refine_yaw=float(config.get('refine_idle_yaw_deg',6.))
        self.refine_state={'enabled':self.refine_enabled,'count':0,'reason':'disabled','applied':False,'idle_s':0.,
                           'checked_at':None,'applied_at':None,'shift_m':None,'shift_deg':None,'score':None,'base_score':None}
        self._still_since=None;self._last_refine_at=0.
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        # RViz 俯视视角跟随小车：发布一个只有平移、不旋转的辅助坐标系，地图始终朝上。
        self.view_broadcaster=TransformBroadcaster(self) if config.get('rviz_follow',True) else None
        self.odom_frame='odom_combined'
        self.subs=[]
        def sub(typ,topic,callback,qos=qos_profile_sensor_data):
            self.subs.append(self.create_subscription(typ,topic,callback,qos))
        sub(OccupancyGrid,'/map',self.on_map,10)
        # Also supports Nav2's transient-local map published before this bridge joins.
        sub(OccupancyGrid,'/map',self.on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        sub(LaserScan,'/scan',self.on_scan)
        sub(Odometry,'/odom_combined',self.on_odom)
        sub(Imu,'/imu/data_raw',self.on_imu)
        sub(RosImage,config['camera_topic'],self.on_camera)
        sub(PoseWithCovarianceStamped,'/amcl_pose',self.on_amcl,10)
        sub(RosPath,'/plan',self.on_plan,10)
        sub(Float32,'/PowerVoltage',self.on_voltage,10)
        sub(BatteryState,'/battery_state',self.on_battery)
        sub(Bool,'/robot_charging_flag',self.on_charging,10)
        sub(Float32,'/robot_charging_current',self.on_current,10)
        sub(Odometry,'/odom',self.on_raw_odom)
        sub(Twist,'/cmd_vel',self.on_command,10)
        self.initial_pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.stop_pub=self.create_publisher(Twist,'/cmd_vel',10)
        # “关避障/贴线巡航”开关：通过参数服务实时改代价地图（不用重启导航栈）
        try:
            from rcl_interfaces.srv import SetParameters
            self.costmap_param_clients=[self.create_client(SetParameters,n) for n in
                                        ('/global_costmap/global_costmap/set_parameters','/local_costmap/local_costmap/set_parameters')]
        except Exception:
            self.costmap_param_clients=[]
        self.serial_state={}
        self.avoidance=bool(config.get('cruise_avoidance',True))
        self.avoidance_state={'enabled':self.avoidance}
        self.speed_pub=self.create_publisher(SpeedLimit,'/speed_limit',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.nav=ActionClient(self,NavigateThroughPoses,'/navigate_through_poses')
        self.planner=ActionClient(self,ComputePathThroughPoses,'/compute_path_through_poses')
        self.nomotion=self.create_client(Empty,'/request_nomotion_update')
        self.global_localizer=self.create_client(Empty,'/reinitialize_global_localization')
        self.amcl_lifecycle=self.create_client(GetState,'/amcl/get_state')
        self.localization_epoch=0
        self.timer=self.create_timer(.2,self.update_tf)
        self.view_timer=self.create_timer(.2,self.publish_view_frame)
        self.refine_timer=self.create_timer(1.,self.idle_refine_tick)
        self.record_timer=self.create_timer(.2,self.record_tick)
        self.skip_timer=self.create_timer(.5,self.mission_skip_tick)
        self.stuck_timer=self.create_timer(.5,self.stuck_tick)
        self.relocalize_timer=self.create_timer(.5,self.mission_relocalize_tick)
        self.aruco_timer=self.create_timer(.5,self.aruco_tick)
        self.speed_timer=self.create_timer(1.,self.publish_speed)
        self.spin_executor=MultiThreadedExecutor(num_threads=3);self.spin_executor.add_node(self)
        self.thread=threading.Thread(target=self._spin,daemon=True);self.thread.start()

    def _spin(self):
        try:
            self.spin_executor.spin()
        except Exception:
            import logging
            logging.exception('ROS executor failed; stopping before supervisor restart')
            try:self._publish_stop_now()
            except Exception:pass
            # systemd kills the whole service cgroup, then starts in idle mode.
            # Never keep a working HTTP server with a dead ROS receiver.
            os._exit(1)

    def on_voltage(self,msg):
        if math.isfinite(msg.data) and msg.data>0:
            with self.lock:self.battery_voltage=float(msg.data);self.voltage_at=time.time()

    def on_charging(self,msg):
        with self.lock:self.charging=bool(msg.data);self.charging_at=time.time()

    def on_current(self,msg):
        if math.isfinite(msg.data):
            with self.lock:self.charging_current=float(msg.data);self.current_at=time.time()

    def on_battery(self,msg):
        def valid(v):return float(v) if math.isfinite(v) else None
        with self.lock:
            self.bms={'percentage':100*msg.percentage if math.isfinite(msg.percentage) and 0<=msg.percentage<=1 else None,
                      'current_a':valid(msg.current),'temperature_c':valid(msg.temperature),'capacity_ah':valid(msg.capacity),'charge_ah':valid(msg.charge),'power_supply_status':int(msg.power_supply_status)}
            self.bms_at=time.time()

    def on_raw_odom(self,msg):
        with self.lock:
            self.raw_odom={'pose':position(msg.pose.pose),'linear_mps':msg.twist.twist.linear.x,'lateral_mps':msg.twist.twist.linear.y,'angular_rps':msg.twist.twist.angular.z,'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id}
            self.raw_odom_at=time.time()

    def on_command(self,msg):
        with self.lock:
            self.commanded={'linear_mps':msg.linear.x,'angular_rps':msg.angular.z};self.commanded_at=time.time()

    def on_map(self,msg):
        stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec,msg.info.width,msg.info.height)
        with self.lock:
            if getattr(self,'_last_map_stamp',None)==stamp:return
            self._last_map_stamp=stamp
        grid=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width).copy()
        meta={'width':msg.info.width,'height':msg.info.height,'resolution':msg.info.resolution,
              'origin':position(msg.info.origin),'frame_id':msg.header.frame_id,'bounds':bounds(grid)}
        png=map_image(grid)
        with self.lock:
            self.map_revision+=1;meta['revision']=self.map_revision
            self.grid=grid;self.map_meta=meta;self.map_png=png;self.map_at=time.time()

    def on_scan(self,msg):
        with self.lock:self.last_scan=msg;self.scan_at=time.time();self.scan_times.append(self.scan_at)

    def on_odom(self,msg):
        with self.lock:
            self.odom_frame=msg.header.frame_id or self.odom_frame
            self.odom={'linear_mps':msg.twist.twist.linear.x,'angular_rps':msg.twist.twist.angular.z,'pose':position(msg.pose.pose)}
            self.odom_at=time.time()

    def on_imu(self,msg):
        q=msg.orientation
        roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
        pitch=math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))
        imu={'roll_deg':math.degrees(roll),'pitch_deg':math.degrees(pitch),'yaw_deg':math.degrees(yaw(q)),
             'angular_velocity':{'x':msg.angular_velocity.x,'y':msg.angular_velocity.y,'z':msg.angular_velocity.z},
             'linear_acceleration':{'x':msg.linear_acceleration.x,'y':msg.linear_acceleration.y,'z':msg.linear_acceleration.z}}
        with self.lock:
            now=time.time();self.imu=imu;self.imu_at=now
            if not self.imu_history or now-self.imu_history[-1][0]>.4:self.imu_history.append([now,imu['roll_deg'],imu['pitch_deg'],imu['yaw_deg']])

    def on_camera(self,msg):
        now=time.time()
        if now-self._camera_encode_at<1/self.config['video_fps']:return
        self._camera_encode_at=now
        try:
            frame=self.cv.imgmsg_to_cv2(msg,'bgr8')
            if frame.shape[1]>800:frame=cv2.resize(frame,(800,int(frame.shape[0]*800/frame.shape[1])))
            ok,encoded=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,75])
            if ok:
                with self.lock:self.camera_jpeg=encoded.tobytes();self.camera_at=now;self.camera_times.append(now);self.camera_size={'width':int(frame.shape[1]),'height':int(frame.shape[0])}
        except Exception as e:self.get_logger().warning(f'Camera conversion: {e}')

    def on_amcl(self,msg):
        with self.lock:self.amcl={'pose':position(msg.pose.pose),'covariance':list(msg.pose.covariance)};self.amcl_at=time.time()

    def on_plan(self,msg):
        with self.lock:self.plan=[position(p.pose) for p in msg.poses[::max(1,len(msg.poses)//500)]]

    def publish_view_frame(self):
        if not self.view_broadcaster:return
        with self.lock:odom=self.odom;frame=self.odom_frame;mode=self.mode
        if not odom:return
        try:
            now=self.get_clock().now().to_msg()
            if mode=='idle':
                # 待命时底盘不发布任何 map→* 变换，RViz 固定坐标系取 map 会无帧可用；
                # 这里补一个恒等变换，建图/巡航时由 gmapping/AMCL 接管，本函数不再发。
                anchor=TransformStamped();anchor.header.stamp=now
                anchor.header.frame_id='map';anchor.child_frame_id=frame
                anchor.transform.rotation.w=1.0
                self.view_broadcaster.sendTransform(anchor)
            t=TransformStamped();t.header.stamp=now
            t.header.frame_id=frame;t.child_frame_id='console_view'
            t.transform.translation.x=float(odom['pose']['x']);t.transform.translation.y=float(odom['pose']['y']);t.transform.translation.z=0.0
            t.transform.rotation.w=1.0
            self.view_broadcaster.sendTransform(t)
        except Exception as e:self.get_logger().warning(f'view frame: {e}')

    def update_tf(self):
        try:
            tf=self.buffer.lookup_transform('map','base_footprint',Time())
            age=(self.get_clock().now().nanoseconds-(tf.header.stamp.sec*10**9+tf.header.stamp.nanosec))/1e9
            if age>1.5 or age < -2:raise RuntimeError('Stale transform')
            t=tf.transform.translation;q=tf.transform.rotation
            pose={'x':t.x,'y':t.y,'yaw':yaw(q)}
            with self.lock:self.pose=pose;self.pose_at=time.time()
        except Exception:
            with self.lock:self.match=None;self.pose_at=0;self.scan_points=[]
            return
        with self.lock:scan=self.last_scan;grid=self.grid;meta=self.map_meta;scan_at=self.scan_at
        if scan is None or time.time()-scan_at>2:return
        try:
            tf=self.buffer.lookup_transform('map',scan.header.frame_id,Time.from_msg(scan.header.stamp),timeout=Duration(seconds=.08))
        except Exception:
            with self.lock:self.scan_points=[];self.match=None
            return
        a=yaw(tf.transform.rotation);t=tf.transform.translation;points=[];hits=0;total=0
        step=max(1,len(scan.ranges)//220)
        for i in range(0,len(scan.ranges),step):
            r=scan.ranges[i]
            if not math.isfinite(r) or not scan.range_min<=r<=scan.range_max:continue
            angle=a+scan.angle_min+i*scan.angle_increment
            x=t.x+r*math.cos(angle);y=t.y+r*math.sin(angle);points.append([x,y])
            if meta is not None and grid is not None:
                gx,gy=world_to_cell(meta,x,y);radius=max(1,int(.18/meta['resolution']))
                if 0<=gx<meta['width'] and 0<=gy<meta['height']:
                    total+=1
                    if np.any(grid[max(0,gy-radius):gy+radius+1,max(0,gx-radius):gx+radius+1]>50):hits+=1
        with self.lock:self.scan_points=points;self.match=hits/total if total>=20 else None;self.match_at=time.time()

    def snapshot_map(self):
        with self.lock:return (dict(self.map_meta) if self.map_meta else None, self.grid.copy() if self.grid is not None else None, dict(self.pose) if self.pose else None)

    def reset_localization(self):
        self.localization_epoch+=1
        with self.lock:
            self.amcl=None;self.amcl_at=0;self.match=None;self.pose=None;self.pose_at=0;self.match_at=0;self.plan=[];self.scan_points=[]
            self.map_meta=None;self.grid=None;self.map_png=None;self._last_map_stamp=None
            self.odom=None;self.odom_at=0;self.imu=None;self.imu_at=0;self.raw_odom=None;self.raw_odom_at=0
            self.last_scan=None;self.scan_at=0;self.scan_times.clear();self.imu_history.clear()
            self.error=None;self._still_since=None;self._last_refine_at=0
            self.refine_state.update({'reason':'waiting_data','applied':False,'idle_s':0.,'checked_at':None})
        self.buffer.clear()

    async def set_avoidance(self,enabled):
        """开关避障：贴线模式把硬禁区收到车体本身、膨胀收到 0.20 m，并实时下发到两个代价地图。"""
        from rcl_interfaces.msg import Parameter as _P,ParameterValue as _PV,ParameterType as _PT
        if enabled:
            clearance=float(self.config_clearance);inflation=max(clearance*1.5,.45)
        else:
            clearance=.10;inflation=.20      # 贴线模式：致命区≈车体本身
        footprint=footprint_text(circle_footprint(clearance,self.footprint_center_cache or (0.,0.)))
        applied=[]
        for client in self.costmap_param_clients:
            if not client.service_is_ready() and not client.wait_for_service(timeout_sec=1.0):continue
            req=SetParameters.Request()
            req.parameters=[_P(name='footprint',value=_PV(type=_PT.PARAMETER_STRING,string_value=footprint)),
                            _P(name='inflation_layer.inflation_radius',value=_PV(type=_PT.PARAMETER_DOUBLE,double_value=inflation))]
            try:
                fut=client.call_async(req);await ros_future(fut,3.);applied.append(True)
            except Exception:applied.append(False)
        with self.lock:self.avoidance=bool(enabled)
        self.avoidance_state={'enabled':bool(enabled),'clearance':clearance,'inflation':inflation,
                              'services':len(applied),'ok':all(applied) if applied else False}
        return dict(self.avoidance_state)

    def nav_stack_up(self):
        """导航栈活着 = 巡航模式 + 地图已加载 + 雷达在出数（scan 新鲜）。"""
        with self.lock:
            return bool(self.mode=='navigation' and self.map_meta is not None and time.time()-self.scan_at<3)

    def front_clearance(self,limit=1.5,half_angle=30.):
        """前方（±half_angle）最近障碍距离；无雷达数据返回 None。"""
        with self.lock:
            scan=self.last_scan
        if scan is None:return None
        try:
            best=None;angle=scan.angle_min
            for r in scan.ranges:
                deg=math.degrees(angle);angle+=scan.angle_increment
                if abs(deg)>half_angle:continue
                if r is None or r!=r or r<=0 or r>=scan.range_max:continue
                best=r if best is None else min(best,r)
            return best
        except Exception:return None

    def rear_clearance(self,limit=.45):
        """后方（±25°）最近障碍距离；没有雷达数据时返回 None（视为未知，不倒车）。"""
        with self.lock:
            scan=self.last_scan
        if scan is None:return None
        try:
            import math as _m
            best=None
            angle=scan.angle_min
            for r in scan.ranges:
                deg=_m.degrees(angle);angle+=scan.angle_increment
                if abs(abs(deg)-180.)>25.:continue                     # 只看正后方 ±25°
                if r is None or r!=r or r<=0 or r>=scan.range_max:continue
                best=r if best is None else min(best,r)
            return best
        except Exception:return None

    def active_clearance(self):
        """当前生效的硬禁区半径（关避障后≈车体半宽）。"""
        return self.config_clearance if self.avoidance else .10

    async def nudge_clear_of_obstacles(self,max_bursts=14,speed=.06,burst_s=.5):
        """起步位置太靠障碍时先倒车让出空间。

        规划器遇到 "Starting point in lethal space" 会直接失败、BT 只能反复恢复，
        表现就是"停在原地/原地磨轮子"。这里小步倒车（每步约 3 cm），
        每步之后重新判断，直到位置可用或达到上限。"""
        for step in range(max_bursts):
            with self.lock:
                pose=self.pose;meta=self.map_meta;grid=self.grid
            if not pose or not start_conflict(meta,grid,pose,self.active_clearance()):
                return step
            rear=self.rear_clearance()
            # 倒车前必须先看后方：没有雷达数据或后方不足 0.45 m 就不倒，宁可报错让用户挪车
            if rear is None or rear<.45:
                self.get_logger().warning(f'后方净空不足（{rear}），放弃自动倒车让位')
                return step
            twist=Twist();twist.linear.x=-abs(speed)          # 阿克曼底盘支持倒车
            for _ in range(max(1,int(burst_s*10))):
                self.stop_pub.publish(twist);await asyncio.sleep(.1)
            self.stop_pub.publish(Twist());await asyncio.sleep(.4)
        return max_bursts

    def _publish_stop_now(self,count=3,gap=.06):
        """同步版停车指令（用于 ROS 定时器回调）：条数少、间隔均匀，避免瞬时突发。"""
        for i in range(max(1,count)):
            self.stop_pub.publish(Twist())
            if i+1<max(1,count):time.sleep(gap)

    async def _publish_stop(self,count=3,gap=.06):
        """异步版停车指令：条数少、间隔均匀，避免瞬时突发压垮下位机串口。"""
        for i in range(max(1,count)):
            self.stop_pub.publish(Twist())
            if i+1<max(1,count):await asyncio.sleep(gap)

    def serial_snapshot(self):
        """抓一份"谁在占用底盘串口"的现场，用于定位串口掉线。

        底盘串口是 /dev/wheeltec_controller（ttyACM0，IMU 也从这条走），
        车上另有 ip_serial_sender.service（把 IP 从串口发出）等会开串口的程序，
        一旦有第二个进程打开同一串口，驱动就会抛
        SerialException: device reports readiness to read but returned no data。
        """
        import subprocess
        def run(cmd):
            try:
                r=subprocess.run(cmd,shell=True,capture_output=True,text=True,timeout=6)
                return (r.stdout or '')+(r.stderr or '')
            except Exception as e:return f'<{e}>'
        info={
            'device':run('ls -l /dev/wheeltec_controller /dev/ttyACM* 2>&1 | head -8'),
            'holders':run('fuser -v /dev/wheeltec_controller /dev/ttyACM0 2>&1 | head -20'),
            'open_fds':run("for p in /proc/[0-9]*; do for f in $p/fd/*; do t=$(readlink $f 2>/dev/null); case \"$t\" in *ttyACM*|*wheeltec*) echo \"${p#/proc/} $(cat $p/comm 2>/dev/null) -> $t\";; esac; done; done 2>/dev/null | head -20"),
            'ip_serial':run('systemctl is-active ip_serial_sender 2>&1; systemctl status ip_serial_sender --no-pager 2>&1 | head -12'),
            'usb_recent':run('dmesg 2>/dev/null | tail -40 | grep -iE "tty|usb|acm|disconnect|reset" | tail -15'),
            'driver_log':run('tail -25 /home/wheeltec/mumai-console/runtime/logs/robot.log 2>/dev/null | grep -aiE "serial|wheeltec_robot|SerialException" | tail -10'),
        }
        with self.lock:self.serial_state={k:v[-1200:] for k,v in info.items()}
        return dict(self.serial_state)

    def action_ready(self,client,name):
        """动作服务是否可用（server_is_ready 或主动 wait_for_server 探测）。

        注意 rclpy 的 Node 没有 get_action_names_and_types()，早先那版回退分支等于永远 False。"""
        try:
            if client.server_is_ready():return True
        except Exception:pass
        try:return bool(client.wait_for_server(timeout_sec=.5))
        except Exception:return False

    def localized(self):
        with self.lock:
            return bool(self.mode=='navigation' and self.amcl and self.pose and time.time()-self.scan_at<2
                        and time.time()-self.match_at<2 and time.time()-self.pose_at<2
                        and self.match is not None and self.match>=.45
                        and max(self.amcl['covariance'][0],self.amcl['covariance'][7])<.5
                        and self.amcl['covariance'][35]<.5)

    def pose_msg(self,p,age=0.):
        # 时间戳回拨一点：AMCL 会把 /initialpose 通过 TF 变换到 odom 帧，
        # 用“当前时刻”时常因 TF 还没到而报 "extrapolation into the future" 并丢弃该位姿。
        msg=PoseStamped();msg.header.frame_id='map'
        stamp=self.get_clock().now()-Duration(seconds=max(0.,age))
        msg.header.stamp=stamp.to_msg()
        msg.pose.position.x=p['x'];msg.pose.position.y=p['y'];msg.pose.orientation.z=math.sin(p['yaw']/2);msg.pose.orientation.w=math.cos(p['yaw']/2)
        return msg

    # ---- 人工定位吸附微调 -------------------------------------------------
    def _likelihood(self):
        """按地图版本缓存似然表：离障碍物越近分数越高。"""
        with self.lock:grid=self.grid;rev=self.map_revision
        if grid is None:return None
        if self._like is not None and self._like_rev==rev:return self._like
        self._like=grid_likelihood(grid,self.map_meta['resolution']);self._like_rev=rev
        return self._like

    def _scan_in_base(self,max_points=240):
        """最近一帧雷达在 base_footprint 下的点，用于给候选位姿打分。"""
        with self.lock:scan,scan_at=self.last_scan,self.scan_at
        if scan is None or time.time()-scan_at>1.0:return None
        try:
            tf=self.buffer.lookup_transform('base_footprint',scan.header.frame_id,Time.from_msg(scan.header.stamp),timeout=Duration(seconds=.06))
        except Exception:return None
        a=yaw(tf.transform.rotation);t=tf.transform.translation
        step=max(1,len(scan.ranges)//max_points);pts=[]
        for i in range(0,len(scan.ranges),step):
            r=scan.ranges[i]
            if not math.isfinite(r) or not scan.range_min<=r<=scan.range_max:continue
            ang=scan.angle_min+i*scan.angle_increment
            pts.append((r*math.cos(ang),r*math.sin(ang)))
        if len(pts)<24:return None
        arr=np.asarray(pts,dtype=np.float32)
        ca,sa=math.cos(a),math.sin(a)
        return np.stack((arr[:,0]*ca-arr[:,1]*sa+t.x,arr[:,0]*sa+arr[:,1]*ca+t.y),axis=1)

    def snap_pose(self,p,max_shift=.35,max_yaw_deg=12.):
        """人工定位吸附：先移出障碍/未知区，再按雷达与地图的吻合度做小范围微调。"""
        with self.lock:grid=self.grid;meta=self.map_meta
        pts=self._scan_in_base() if (grid is not None and self.mode=='navigation') else None
        like=self._likelihood() if pts is not None else None
        pose={k:float(p[k]) for k in ('x','y','yaw')}
        if self.mode!='navigation' or grid is None or meta is None:
            return {'applied':False,'free_shift_m':0.0,'shift_m':0.0,'shift_deg':0.0,'score':None,'samples':0,'reason':'no_map','pose':pose}
        return core_snap_pose(meta,grid,pts,pose,max_shift=max_shift,max_yaw_deg=max_yaw_deg,like=like)

    def localize(self,p,covariance=None):
        if self.mode!='navigation':raise ConsoleError('NOT_NAVIGATING','请先加载巡航地图')
        if self.mission['state'] in ('running','accepting','pausing','paused','stopping'):raise ConsoleError('MISSION_ACTIVE','请先停止巡航')
        self.localization_epoch+=1
        msg=PoseWithCovarianceStamped();pose=self.pose_msg(p,age=.05);msg.header=pose.header;msg.pose.pose=pose.pose
        var_xy,var_yaw=covariance or (.25,.0685)
        msg.pose.covariance[0]=var_xy;msg.pose.covariance[7]=var_xy;msg.pose.covariance[35]=var_yaw
        self.amcl=None;self.error=None
        self.initial_pub.publish(msg)
        if self.nomotion.service_is_ready():self.nomotion.call_async(Empty.Request())

    async def refine_localization(self):
        ticket=self.localization_epoch
        for _ in range(20):
            await asyncio.sleep(.5)
            if ticket!=self.localization_epoch or self.mode!='navigation':return
            if self.nomotion.service_is_ready():await ros_future(self.nomotion.call_async(Empty.Request()),3)
            if self.localized():return

    async def auto_localize(self):
        if self.mode!='navigation':raise ConsoleError('NOT_NAVIGATING','请先加载巡航地图')
        if self.mission['state'] in ('running','accepting','pausing','paused','stopping'):raise ConsoleError('MISSION_ACTIVE','请先停止巡航')
        self.localization_epoch+=1;ticket=self.localization_epoch
        self.error=None
        # Services exist during configure, before AMCL is safe to initialize.
        # Wait for the lifecycle transition AND receipt of the new map.
        for _ in range(90):
            if ticket!=self.localization_epoch or self.mode!='navigation':return
            if self.map_meta and self.amcl_lifecycle.service_is_ready():
                try:
                    state=await ros_future(self.amcl_lifecycle.call_async(GetState.Request()),2)
                    if (state.current_state.id==State.PRIMARY_STATE_ACTIVE
                            and self.global_localizer.service_is_ready()):break
                except ConsoleError:pass
            await asyncio.sleep(.5)
        else:raise ConsoleError('NAV_NOT_READY','定位模块未就绪，请重新加载地图')
        if ticket!=self.localization_epoch or self.mode!='navigation':return
        self.amcl=None
        try:await ros_future(self.global_localizer.call_async(Empty.Request()))
        except ConsoleError as e:
            raise ConsoleError('LOCALIZATION_UNAVAILABLE','定位服务无响应，请重新加载地图',503) from e
        for attempt in range(60):
            if ticket!=self.localization_epoch or self.mode!='navigation':return
            # A guarded early AMCL request returns Empty even if its map callback
            # has not run. Retry only until the first pose arrives, never reset
            # an already converging particle filter.
            if attempt in (10,20) and self.amcl is None:
                await ros_future(self.global_localizer.call_async(Empty.Request()))
            if self.nomotion.service_is_ready():await ros_future(self.nomotion.call_async(Empty.Request()),3)
            await asyncio.sleep(.5)
            if self.localized():return
        self.error='自动定位未收敛，可使用人工定位'

    # ---- 航迹记录（遥控教学） ---------------------------------------------
    def record_command(self,action):
        """start / stop / clear：记录遥控行驶轨迹，每 record_step_m 米一个航点。"""
        with self.lock:
            if action=='start':
                if not self.pose:raise ConsoleError('NOT_LOCALIZED','还没有可用位姿，请先建图或定位')
                self.record={'active':True,'count':0,'distance_m':0.,'points':[],'started_at':time.time()}
            elif action=='stop':
                self.record['active']=False
            elif action=='clear':
                self.record={'active':False,'count':0,'distance_m':0.,'points':[],'started_at':None}
            else:raise ConsoleError('INVALID_ARGUMENT','记录动作无效',422)
            return dict(self.record,points=list(self.record['points']))

    def record_tick(self):
        with self.lock:
            if not self.record['active']:return
            pose=self.pose if time.time()-self.pose_at<1. else None
            points=self.record['points']
            if len(points)>=self.record_max:
                self.record['active']=False;return
            step=record_step(points,pose,self.record_step_m)
            if not step:return
            if points:self.record['distance_m']=round(self.record['distance_m']+math.hypot(step[0]-points[-1][0],step[1]-points[-1][1]),2)
            points.append(step);self.record['count']=len(points)

    def set_idle_refine(self,enabled):
        self.refine_enabled=bool(enabled)
        if not self.refine_enabled:self._still_since=None
        self.refine_state['enabled']=self.refine_enabled
        if not self.refine_enabled:self.refine_state['reason']='disabled'
        return self.refine_state

    def idle_refine_tick(self):
        """每秒一次：底盘静止足够久时，用当前位姿做一次小窗口吸附校准。"""
        now=time.time()
        with self.lock:
            odom=self.odom;odom_at=self.odom_at;pose=self.pose;pose_at=self.pose_at;match=self.match;match_at=self.match_at
        if not odom or now-odom_at>1.5:
            self._still_since=None
        else:
            linear=odom.get('linear_mps') or 0.;angular=odom.get('angular_rps') or 0.
            if abs(linear)>.02 or abs(angular)>.05:self._still_since=None
            elif self._still_since is None:self._still_since=now
        ok,reason=idle_refine_decision({'enabled':self.refine_enabled,'mode':self.mode,'mission':self.mission['state'],
                                        'linear':odom.get('linear_mps') if odom else None,'angular':odom.get('angular_rps') if odom else None,
                                        'still_since':self._still_since,'last_at':self._last_refine_at,'interval_s':self.refine_interval,
                                        'delay_s':self.refine_delay,'pose_age':(now-pose_at) if pose else None,'match':match},
                                       now)
        if not odom or now-odom_at>1.5:reason='waiting_data'
        self.refine_state.update({'reason':reason,'idle_s':round(now-self._still_since,1) if self._still_since else 0.,
                                  'time':now,'match':match})
        if not ok or not pose:return
        self._last_refine_at=now
        try:result=self.snap_pose({'x':pose['x'],'y':pose['y'],'yaw':pose['yaw']},max_shift=self.refine_shift,max_yaw_deg=self.refine_yaw)
        except Exception as e:
            self.get_logger().warning(f'idle refine: {e}');self.refine_state.update({'checked_at':now,'applied':False,'reason':'error'});return
        shift=result.get('shift_m') or 0.;dth=abs(result.get('shift_deg') or 0.)
        self.refine_state.update({'checked_at':now,'score':result.get('score'),'base_score':result.get('base_score')})
        if not result.get('applied') or (shift<.005 and dth<.2):
            self.refine_state.update({'applied':False,'reason':result.get('reason') or 'no_gain'});return
        try:self.localize(result['pose'],covariance=(.10,.03))
        except Exception as e:
            self.get_logger().warning(f'idle refine publish: {e}');self.refine_state.update({'applied':False,'reason':'error'});return
        self.refine_state.update({'applied':True,'applied_at':now,'reason':'applied','count':self.refine_state.get('count',0)+1,
                                  'shift_m':shift,'shift_deg':result.get('shift_deg'),'pose':result['pose']})

    def publish_speed(self):
        # Zero means NO_SPEED_LIMIT in Nav2; the API pauses the mission at zero.
        speed=self.speed
        if speed<=0:return
        msg=SpeedLimit();msg.header.stamp=self.get_clock().now().to_msg();msg.percentage=False;msg.speed_limit=speed
        self.speed_pub.publish(msg)

    async def preview(self,points,mode):
        if self.mode!='navigation':raise ConsoleError('NOT_NAVIGATING','请先加载巡航地图')
        if not self.localized():raise ConsoleError('NOT_LOCALIZED','请先完成定位')
        with self.lock:
            points=validate_waypoints(points,self.map_meta,self.grid,mode)
            pose=self.pose;meta=self.map_meta;grid=self.grid
        nudged=0
        if start_conflict(meta,grid,pose,self.active_clearance()):
            # 离墙太近 → 先倒车让位再规划（规划器无法从致命栅格起步）
            nudged=await self.nudge_clear_of_obstacles()
            with self.lock:pose=self.pose;meta=self.map_meta;grid=self.grid
            if start_conflict(meta,grid,pose,self.active_clearance()):
                raise ConsoleError('START_BLOCKED',
                    f'已尝试倒车让位（{nudged*3} cm）但车仍离障碍不足 {self.active_clearance():g} m，请手动挪车后再预览',409)
            points=validate_waypoints(points,meta,grid,mode)
        self.nudged_cm=nudged*3
        self.plan=[]
        # 注意：这台机器上 ROS 2 的动作发现不可靠（新建 ActionClient 的 wait_for_server 也探不到
        # /compute_path_through_poses，尽管 ros2 action list 里有）。所以不拿它当门禁，
        # 直接发目标；真正连不上时下面会给出明确错误。
        if not self.planner.server_is_ready() and not self.planner.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('规划器动作未发现，仍尝试直接发送目标')
        goals=automatic_goals(points,pose,mode)
        route=prune_reached_goals(goals+([goals[0]] if mode=='loop' else []),pose,self.arrival_radius)
        if not route:raise ConsoleError('NO_PATH','起点已覆盖全部航点，请调整航点或先移动小车',409)
        req=ComputePathThroughPoses.Goal();req.goals=[self.pose_msg(p) for p in route]
        req.planner_id='GridBased';req.use_start=False
        handle=await ros_future(self.planner.send_goal_async(req))
        if not handle.accepted:raise ConsoleError('PLAN_REJECTED','路径规划被拒绝')
        try:result=await ros_future(handle.get_result_async(),max(20,min(120,6*len(route))))
        except BaseException:
            handle.cancel_goal_async();raise
        if result.status!=4 or not result.result.path.poses:
            with self.lock:near=points_within(self.scan_points,pose,.40)
            hint='，起点附近有障碍（雷达点过近），请把小车移开一点或清理代价地图后重试' if near>=6 else '，请调整航点位置'
            raise ConsoleError('NO_PATH','无可行的连续前向路径'+hint,409)
        output=[position(v.pose) for v in result.result.path.poses]
        with self.lock:self.plan=output[::max(1,len(output)//1500)]
        return {'path':self.plan,'point_count':len(points)}

    async def start_mission(self,points,mode):
        with self.lock:
            if self.speed<=0:raise ConsoleError('ZERO_SPEED','巡航速度为 0，请先调高速度')
            if self.mission['state'] in ('running','accepting','pausing','paused','stopping'):raise ConsoleError('MISSION_ACTIVE','已有巡航任务')
            if not self.localized():raise ConsoleError('NOT_LOCALIZED','请先完成定位')
            if not self.nav_stack_up():raise ConsoleError('NAV_NOT_READY','导航服务未就绪')
            # 先让位再校验（校验会因起点落在障碍/未知区而拒绝）
            if start_conflict(self.map_meta,self.grid,self.pose,self.active_clearance()):
                await self.nudge_clear_of_obstacles()
                if start_conflict(self.map_meta,self.grid,self.pose,self.active_clearance()):
                    raise ConsoleError('START_BLOCKED',
                        f'已尝试倒车让位但车仍离障碍不足 {self.active_clearance():g} m，请手动挪车后再开始巡航',409)
            points=validate_waypoints(points,self.map_meta,self.grid,mode)
            self.navigation_goals=automatic_goals(points,self.pose,mode)
            # 循环路线从离车最近的航点开始（否则一开跑就横穿全场去 1 号点）；
            # 线性路线仍从第 1 个点开始，保持“按顺序走完”的语义。
            if mode=='loop' and points and self.pose:
                self.route_cursor=min(range(len(points)),
                                      key=lambda i:math.hypot(points[i]['x']-self.pose['x'],points[i]['y']-self.pose['y']))
            else:
                self.route_cursor=0
            self.epoch+=1
            self.mission={'state':'accepting','index':0,'cycle':0,'points':points,'mode':mode,'distance_remaining':None,'skipped':0,'skip_reason':None,'abort_reason':None,'stop_reason':None}
            self.error=None;self.publish_speed();self.mission_trace('start cursor=%d mode=%s'%(self.route_cursor,mode));self._send_nav(self.epoch)
            return dict(self.mission)

    def _batch_goals(self,start):
        """本次下发的目标窗口（规则见 core.plan_batch）。"""
        return plan_batch(self.navigation_goals,start,self.mission['mode'],self.lookahead,self.lookahead_m,self.lookahead_max)

    def _waypoint_index(self):
        return waypoint_index(self.route_cursor,self.mission['mode'],len(self.navigation_goals))

    def _send_nav(self,ticket):
        if ticket!=self.epoch:return
        m=self.mission;start=self.route_cursor
        route=self._batch_goals(start)
        if not route:
            with self.lock:
                m['index']=self._waypoint_index();m['state']='completed';m['distance_remaining']=0.
            return
        req=NavigateThroughPoses.Goal();req.poses=[self.pose_msg(p) for p in route]
        count=len(req.poses)
        self._batch_end=start+count
        def feedback(msg):
            with self.lock:
                if ticket!=self.epoch:return
                remaining=int(msg.feedback.number_of_poses_remaining)
                if 1<=remaining<=count:
                    self.route_cursor=max(self.route_cursor,start+count-remaining)
                    m['index']=self._waypoint_index()
                m['distance_remaining']=remaining_route_distance(self.navigation_goals,start,count,m['mode'],msg.feedback.distance_remaining)
        f=self.nav.send_goal_async(req,feedback_callback=feedback);self.pending_goal=f
        def accepted(f):
            try:handle=f.result()
            except Exception as e:
                if ticket==self.epoch:self.mission['state']='failed';self.error=str(e)
                return
            with self.lock:
                if ticket!=self.epoch:
                    if handle.accepted:handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    self.mission['state']='failed';self.error='导航目标被拒绝';return
                self.goal_handle=handle;self.mission['state']='running'
            handle.get_result_async().add_done_callback(lambda f:self._finished(f,ticket))
        f.add_done_callback(accepted)

    def mission_trace(self,event):
        with self.lock:
            self.trace.append({'t':round(time.time()%10000,1),'event':event,
                               'cursor':self.route_cursor,'idx':self.route_cursor%max(1,len(self.navigation_goals)) if self.navigation_goals else 0,
                               'skipped':self.mission.get('skipped'),'stuck':round(self.stuck_s,1)})
            if len(self.trace)>30:self.trace.popleft()
        try:self.get_logger().info(f'[mission] {event} cursor={self.route_cursor}')
        except Exception:pass

    def stuck_tick(self):
        """车是不是停在原地不动（任务运行中）：供前端显示原因，并在超过阈值时自动跳点。"""
        now=time.time()
        with self.lock:
            if self.mission['state'] not in ('running','accepting'):
                self.stuck_s=0.;self._move_ref=None;self._move_at=None;return
            pose=self.pose if time.time()-self.pose_at<2 else None
        if not pose:
            self.stuck_s=0.;return
        if self._move_ref is None or math.hypot(pose['x']-self._move_ref[0],pose['y']-self._move_ref[1])>.03:
            self._move_ref=(pose['x'],pose['y']);self._move_at=now;self.stuck_s=0.
            self.stuck_skips=max(0,self.stuck_skips-1)      # 有前进就抵消一次跳过额度
            return
        self.stuck_s=now-(self._move_at or now)
        if self.stuck_s<self.stuck_after or self._skipping:return
        # 只有“目标就在旁边却迟迟到不了”才跳过它（例如该点落在禁区里）。
        # 若车离目标还很远却完全不动，说明是整体性问题（被挡住/控制器算不过来），
        # 这时跳点只会把游标推着往前跑、最后绕回起点，反而更糟——所以不跳，只报原因。
        with self.lock:
            goals=self.navigation_goals;n=len(goals)
            loop=self.mission['mode']=='loop'
            target=goals[(self.route_cursor%n)] if (loop and n) else (goals[min(self.route_cursor,n-1)] if n else None)
            pose=self.pose
        near=bool(target and pose and math.hypot(pose['x']-target['x'],pose['y']-target['y'])<=max(self.skip_near,.45))
        if not near or not self.skip_missed:
            if self.stuck_s>self.stuck_after*3:
                with self.lock:
                    self.mission['state']='failed';self.mission['abort_reason']='stuck'
                    self.mission['stop_reason']=(f'车连续 {int(self.stuck_s)} s 完全没有前进（离当前航点还有 '
                                                 f"{math.hypot(pose['x']-target['x'],pose['y']-target['y']):.1f} m）："
                                                 '多为被障碍挡住、路径不可行或控制器算不过来；已停车')
                    self.error='已停车：'+self.mission['stop_reason']
                    handle=self.goal_handle;self.goal_handle=None
                try:
                    if handle:handle.cancel_goal_async()
                    self._publish_stop_now()
                except Exception:pass
            return
        if self.stuck_skips>=self.stuck_max:
            with self.lock:
                self.mission['state']='failed'
                self.mission['abort_reason']='stuck'
                self.mission['stop_reason']=(f'车连续 {int(self.stuck_s)} s 没有前进（已尝试跳过 {self.stuck_skips} 个航点仍不行）：'
                                             '多为前方被挡、路径不可行或控制器算不过来（可关避障或降低巡航速度）')
                self.error='已停车：'+self.mission['stop_reason']
                handle=self.goal_handle;self.goal_handle=None
            try:
                if handle:handle.cancel_goal_async()
                self._publish_stop_now()
            except Exception:pass
            return
        # 卡住 → 跳过当前航点继续（比原地等恢复动作快得多）
        with self.lock:
            self._skipping=True;self._last_skip_at=now;self.stuck_skips+=1
            self.mission['skip_reason']='卡住不动，已跳过'
            self.mission_trace('skip:stuck')
            self._move_ref=None;self._move_at=now;self.stuck_s=0.
            handle=self.goal_handle
        try:handle.cancel_goal_async()
        except Exception:
            with self.lock:self._skipping=False

    def mission_skip_tick(self):
        """当前航点要不要跳过（用户要求：兜圈子就直接去下一个点）。

        三种情况都算：
          A 已经越过该点（点密集时擦肩而过）；
          B 车已停在该点附近却迟迟到不了（比如点落在 30cm 禁区里永远进不去）；
          C 规划出来的路径一开头就朝反方向走（要先掉头/绕圈）。
        动作是取消当前动作并从下一个点续跑，代价是短暂减速，换来不兜圈。"""
        if not self.skip_missed:return
        with self.lock:
            if self._skipping or self.mission['state'] not in ('running','accepting'):return
            if not self.goal_handle or time.time()-self._last_skip_at<3.:return   # 两次跳过至少隔 3 s，避免连环跳点
            pose=self.pose if time.time()-self.pose_at<1. else None
            goals=self.navigation_goals;n=len(goals)
            if not pose or not n:return
            loop=self.mission['mode']=='loop'
            index=self.route_cursor%n if loop else min(self.route_cursor,n-1)
            target=goals[index]
            prev=goals[(index-1)%n] if loop else (goals[index-1] if index else None)
            direction=(target['x']-prev['x'],target['y']-prev['y']) if prev else None
            plan=list(self.plan)
            speed=abs((self.odom or {}).get('linear_mps') or 0.)
            distance=math.hypot(pose['x']-target['x'],pose['y']-target['y'])
            # 停滞判定：以“进入目标附近”那一刻的位置为参考点
            if distance>self.skip_near:self._stall_ref=None;self._stall_since=None
            elif self._stall_ref is None:self._stall_ref=(pose['x'],pose['y']);self._stall_since=time.time()
            stalled=stalled_here(self._stall_ref,{'x':pose['x'],'y':pose['y'],'speed':speed},
                                 time.time()-(self._stall_since or time.time()),self.skip_stall)
            reason=None
            # “越过”判据收紧：不仅要越过横截面 0.35 m，还必须确实在远离该点
            # （否则在循环路线上贴着点旁经过会连跳，把游标一路推回起点）
            if missed_waypoint(pose,target,direction,.35):
                prev_key=(index,)
                was=self._target_distance.get(prev_key)
                self._target_distance={prev_key:distance}
                if was is None or distance>was+.02:reason='越过'
            elif stalled and distance<=self.skip_near:reason='停在点旁到不了'
            if not reason:return
            self._skipping=True;self._last_skip_at=time.time()
            self.mission['skip_reason']=reason
            handle=self.goal_handle
        try:handle.cancel_goal_async()
        except Exception:
            with self.lock:self._skipping=False

    def mission_relocalize_tick(self):
        """航行中定位守护：吻合度持续偏低 → 小窗口吸附并重锚；多次无效 → 安全停车。"""
        now=time.time()
        with self.lock:
            match=self.match if now-self.match_at<2 else None
            pose=self.pose if now-self.pose_at<2 else None
            low=match is not None and match<self.relocalize_match and self.mission['state'] in ('running','accepting','paused')
            if not low:self._match_low_since=None
            elif self._match_low_since is None:self._match_low_since=now
            action,reason=relocalize_decision({'enabled':self.relocalize_enabled,'mission':self.mission['state'],'match':match,
                'low_since':self._match_low_since,'last_at':self._last_relocalize_at,'after_s':self.relocalize_after,
                'interval_s':self.relocalize_interval,'count':self.relocalize_state['count'],'max_count':self.relocalize_max,
                'threshold':self.relocalize_match},now)
            self.relocalize_state.update({'reason':reason,'match':round(match,3) if match is not None else None})
        if action=='wait' or not pose:return
        if action=='stop':
            with self.lock:
                self.mission['state']='failed'
                self.mission['abort_reason']='localization'
                self.error=f'定位持续失准（吻合度 {int((match or 0)*100)}%），已安全停车，请重新定位后再巡航'
                handle=self.goal_handle;self.goal_handle=None;self.pending_goal=None
            try:
                if handle:handle.cancel_goal_async()
                self._publish_stop_now()
            except Exception:pass
            self.relocalize_state.update({'reason':'stopped','at':now})
            return
        # action == 'anchor'：围绕当前位姿做一次小窗口吸附，采纳后重锚 AMCL
        self._last_relocalize_at=now
        try:result=self.snap_pose({'x':pose['x'],'y':pose['y'],'yaw':pose['yaw']},max_shift=.25,max_yaw_deg=8.)
        except Exception as e:
            self.get_logger().warning(f'relocalize: {e}');return
        if not result.get('applied') or (abs(result.get('shift_m') or 0)<.01 and abs(result.get('shift_deg') or 0)<.3):
            self.relocalize_state.update({'reason':'no_gain','at':now});return
        try:self.localize(result['pose'],covariance=(.10,.03))
        except Exception as e:
            self.get_logger().warning(f'relocalize publish: {e}');return
        self.relocalize_state.update({'count':self.relocalize_state['count']+1,'reason':'applied','at':now,
                                      'shift_m':result.get('shift_m'),'shift_deg':result.get('shift_deg'),
                                      'match':round(match,3) if match is not None else None,'score':result.get('score')})

    def _finished(self,f,ticket):
        with self.lock:
            if ticket!=self.epoch:return
            self.goal_handle=None
            self.pending_goal=None
            try:status=f.result().status
            except Exception:status=6
            if status!=4:
                if self._skipping:
                    # 主动跳过被越过的航点：推进游标后从下一个点续跑
                    self._skipping=False
                    self.goal_handle=None;self.pending_goal=None;self.route_cursor+=1
                    self.mission['skipped']=self.mission.get('skipped',0)+1
                    self.mission['index']=self._waypoint_index();self.mission['state']='accepting'
                    self._send_nav(self.epoch)
                    return
                # A local safety stop already has the more precise failure reason.
                if self.mission['state']=='failed' and self.mission.get('stop_reason'):
                    return
                self.mission['state']='failed'
                with self.lock:
                    blocked=start_conflict(self.map_meta,self.grid,self.pose,self.active_clearance())
                    empty=not self.plan
                    match=self.match;skipped=self.mission.get('skipped') or 0
                    skip_reason=self.mission.get('skip_reason')
                self.mission['abort_reason']='start_blocked' if blocked else ('no_path' if empty else 'aborted')
                self.mission['stop_reason']=mission_stop_reason(start_blocked=blocked,plan_empty=empty,match=match,
                                                               status=status,skipped=skipped,skip_reason=skip_reason,
                                                               clearance=self.active_clearance())
                self.error='已停车：'+self.mission['stop_reason']
                return
            self.pending_goal=None
            # 一批可能覆盖多个航点：整批走完才推进游标
            # Feedback already advances route_cursor within this submitted batch.
            self.route_cursor=max(self.route_cursor,self._batch_end)
            n=len(self.navigation_goals)
            m=self.mission
            if m['mode']=='loop':
                m['cycle']=lap_number(self.route_cursor,n)
                m['index']=self._waypoint_index()
                self.mission_trace('batch ok -> cursor=%d'%self.route_cursor)
                self._send_nav(ticket)
            elif self.route_cursor>=n:
                self.route_cursor=n;m['index']=max(0,n-1)
                m['state']='completed';m['distance_remaining']=0.
            else:
                m['index']=self._waypoint_index()
                self._send_nav(ticket)

    # ---- 相机 ArUco 标签定位 ----------------------------------------------
    def aruco_tick(self):
        """看到已知地图位姿的标签时，用相机给出绝对位姿并重锚 AMCL。

        标签地图位姿由现场标定得到（把车停在能看到标签的位置点“记录标签位置”）；
        连续 confirm_frames 帧一致、且与当前 belief 相差不超过 max_jump 才采纳。"""
        if not self.aruco['enabled'] or not self.aruco['marker_pose']:return
        now=time.time()
        try:
            tf=self.buffer.lookup_transform('base_footprint',self.aruco['frame'],Time(),timeout=Duration(seconds=.05))
        except Exception:
            with self.lock:
                self.aruco_state.update({'seeing':False,'reason':'no_marker'})
            self._aruco_streak=0;self._aruco_seen=None
            return
        t=tf.transform.translation;q=tf.transform.rotation
        marker_in_robot={'x':t.x,'y':t.y,'yaw':yaw(q)}
        with self.lock:
            pose=self.pose if time.time()-self.pose_at<2 else None
            candidate=robot_pose_from_marker(self.aruco['marker_pose'],marker_in_robot)
            self.aruco_state.update({'seeing':True,'marker_in_robot':{k:round(v,3) for k,v in marker_in_robot.items()}})
            if not candidate or not pose:
                self.aruco_state['reason']='no_pose';return
            if not aruco_jump_ok(pose,candidate,self.aruco['max_jump']):
                self.aruco_state['reason']='jump_rejected';self._aruco_streak=0;return
            if not aruco_consistent(self._aruco_seen,candidate):
                self._aruco_seen=candidate;self._aruco_streak=1;self.aruco_state['reason']='confirming';return
            self._aruco_seen=candidate;self._aruco_streak+=1
            self.aruco_state['reason']='confirming'
            if self._aruco_streak<self.aruco['confirm_frames']:return
            if now-self._aruco_last_at<1.:return
            self._aruco_last_at=now
            shift=math.hypot(candidate['x']-pose['x'],candidate['y']-pose['y'])
            if shift<.03 and abs(math.degrees(candidate['yaw']-pose['yaw']))<.5:
                self.aruco_state['reason']='already_aligned';return
            self.localize(candidate,covariance=(.05,.02))
            self.aruco_state.update({'count':self.aruco_state['count']+1,'reason':'applied','at':now,
                                     'shift_m':round(shift,3),'shift_deg':round(math.degrees(candidate['yaw']-pose['yaw']),1)})
            self._aruco_seen=candidate;self._aruco_streak=0

    def aruco_capture(self):
        """标定：用当前 belief + 标签相对车体的位姿，反算标签在地图中的位姿。"""
        try:
            tf=self.buffer.lookup_transform('base_footprint',self.aruco['frame'],Time(),timeout=Duration(seconds=.05))
        except Exception:raise ConsoleError('ARUCO_NOT_SEEN','相机现在看不到标签，请把车对准标签再记录',409)
        t=tf.transform.translation;q=tf.transform.rotation
        with self.lock:pose=self.pose if time.time()-self.pose_at<2 else None;match=self.match
        if not pose:raise ConsoleError('NOT_LOCALIZED','当前没有可用位姿，请先完成定位',409)
        if (match or 0)<.8:raise ConsoleError('LOCALIZATION_WEAK',f'当前吻合度仅 {int((match or 0)*100)}%，标定需要 ≥80%，请先确认定位准确',409)
        marker=marker_pose_from_robot(pose,{'x':t.x,'y':t.y,'yaw':yaw(q)})
        self.aruco['marker_pose']={'x':round(marker['x'],3),'y':round(marker['y'],3),'yaw':round(marker['yaw'],4)}
        self.aruco_state.update({'configured':True,'reason':'configured'})
        return dict(self.aruco['marker_pose'])

    async def stop(self,pause=False):
        async with self.cancel_lock:
            with self.lock:
                self.epoch+=1;handle=self.goal_handle;pending=self.pending_goal
                self.goal_handle=None;self.pending_goal=None
                self.mission['state']='pausing' if pause else 'stopping'
            try:
                if pending and not handle:
                    handle=await ros_future(pending,5)
                if handle and handle.accepted:
                    result=handle.get_result_async()
                    if not result.done():await ros_future(handle.cancel_goal_async(),5)
                    await ros_future(result,6)
            except Exception as e:
                self.error='导航取消未确认';self.mission['state']='failed'
                raise ConsoleError('CANCEL_UNCONFIRMED','导航取消未确认，需要关闭导航进程',503) from e
            finally:
                for _ in range(5):self.stop_pub.publish(Twist());await asyncio.sleep(.05)
            with self.lock:self.mission['state']='paused' if pause else 'stopped'

    def resume(self):
        with self.lock:
            if self.speed<=0:raise ConsoleError('ZERO_SPEED','巡航速度为 0，请先调高速度')
            if self.mission['state']!='paused':raise ConsoleError('NOT_PAUSED','任务未暂停')
            if not self.localized():raise ConsoleError('NOT_LOCALIZED','定位未就绪')
            if not self.nav_stack_up():raise ConsoleError('NAV_NOT_READY','导航服务未就绪')
            self.epoch+=1;self.mission['state']='accepting';self._send_nav(self.epoch)

    def snapshot(self):
        now=time.time()
        def hz(ts):return round((len(ts)-1)/(ts[-1]-ts[0]),1) if len(ts)>1 and ts[-1]>ts[0] else None
        def age(t):return round(now-t,2) if t else None
        with self.lock:
            return {'mode':self.mode,'uptime_s':int(now-self.started_at),'map':self.map_meta,
                    'pose':self.pose if now-self.pose_at<2 else None,'scan_points':self.scan_points,'path':self.plan,
                    # 就绪判断只用能直接观测到的信号：动作服务发现（ActionClient）在
                    # 服务端重启后经常一直为 False，界面会卡在“导航初始化”导致预览点不了。
                    # 导航栈活着 = 巡航模式 + 地图已加载 + 雷达在出数。
                    'navigation':{'ready':self.nav_stack_up(),'planner_ready':self.nav_stack_up()},
                    'localization':{'ready':self.localized(),'match':self.match,'refine':dict(self.refine_state),'relocalize':dict(self.relocalize_state),'aruco':dict(self.aruco_state),'amcl_received':self.amcl is not None,'match_age_s':age(self.match_at),'covariance':{'x_m2':self.amcl['covariance'][0],'y_m2':self.amcl['covariance'][7],'yaw_rad2':self.amcl['covariance'][35]} if self.amcl else None},
                    'velocity':self.odom if now-self.odom_at<2 else None,
                    'imu':self.imu if now-self.imu_at<2 else None,'imu_history':list(self.imu_history),
                    'lidar':{'state':'online' if now-self.scan_at<2 else 'offline','hz':hz(self.scan_times) if now-self.scan_at<2 else None,'age_s':age(self.scan_at)},
                    'camera':{'state':'online' if now-self.camera_at<3 else 'offline','size':self.camera_size,'fps':hz(self.camera_times) if now-self.camera_at<3 else None,'age_s':age(self.camera_at)},
                    'commanded':self.commanded if now-self.commanded_at<3 else None,'front_clearance_m':self.front_clearance(),
                    'mission':{**dict(self.mission),'stuck_s':round(self.stuck_s,1)},'mission_trace':list(self.trace)[-8:],'record':{'active':self.record['active'],'count':self.record['count'],'distance_m':self.record['distance_m'],'points':self.record['points']},'speed_mps':self.speed,'battery_voltage':self.battery_voltage if now-self.voltage_at<5 else None,
                    'battery':{'state':'online' if now-self.voltage_at<5 or now-self.bms_at<5 else 'offline','voltage_v':self.battery_voltage if now-self.voltage_at<5 else None,'age_s':age(self.voltage_at),'percentage':self.bms['percentage'] if self.bms and now-self.bms_at<5 else None,'charging':self.charging if now-self.charging_at<5 else None,'charging_current_a':self.charging_current if now-self.current_at<5 else None,'bms':self.bms if now-self.bms_at<5 else None},
                    'chassis':{'state':'online' if now-self.raw_odom_at<2 else 'offline','model':'mini_akm','drive_type':'ackermann','age_s':age(self.raw_odom_at),'odometry':self.raw_odom if now-self.raw_odom_at<2 else None,'commanded_velocity':self.commanded if now-self.commanded_at<2 else None,'command_age_s':age(self.commanded_at)},'error':self.error}

    def close(self):
        self.spin_executor.shutdown(timeout_sec=3);self.destroy_node();rclpy.shutdown()
