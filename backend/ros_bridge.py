"""ROS 2 adapter. All navigation goes through Nav2 actions, never pixel clicks."""
import asyncio
import io
import math
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
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav2_msgs.msg import SpeedLimit
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from cv_bridge import CvBridge
import cv2
from core import ConsoleError, bounds, grid_likelihood, map_image, world_to_cell, snap_pose as core_snap_pose


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
        self._like=None;self._like_rev=None
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
        self.speed_pub=self.create_publisher(SpeedLimit,'/speed_limit',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.nomotion=self.create_client(Empty,'/request_nomotion_update')
        self.global_localizer=self.create_client(Empty,'/reinitialize_global_localization')
        self.localization_epoch=0
        self.timer=self.create_timer(.2,self.update_tf)
        self.view_timer=self.create_timer(.2,self.publish_view_frame)
        self.speed_timer=self.create_timer(1.,self.publish_speed)
        self.spin_executor=MultiThreadedExecutor(num_threads=3);self.spin_executor.add_node(self)
        self.thread=threading.Thread(target=self.spin_executor.spin,daemon=True);self.thread.start()

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
        with self.lock:odom=self.odom;frame=self.odom_frame
        if not odom:return
        try:
            t=TransformStamped();t.header.stamp=self.get_clock().now().to_msg()
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
            with self.lock:self.match=None;self.pose_at=0
            return
        with self.lock:scan=self.last_scan;grid=self.grid;meta=self.map_meta;scan_at=self.scan_at
        if scan is None or time.time()-scan_at>2:return
        try:
            tf=self.buffer.lookup_transform('map',scan.header.frame_id,Time.from_msg(scan.header.stamp),timeout=Duration(seconds=.08))
        except Exception:return
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
        self.buffer.clear()

    def localized(self):
        with self.lock:
            return bool(self.mode=='navigation' and self.amcl and self.pose and time.time()-self.scan_at<2
                        and time.time()-self.match_at<2 and time.time()-self.pose_at<2
                        and self.match is not None and self.match>=.45
                        and max(self.amcl['covariance'][0],self.amcl['covariance'][7])<.5
                        and self.amcl['covariance'][35]<.5)

    def pose_msg(self,p):
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
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

    def snap_pose(self,p):
        """人工定位吸附：先移出障碍/未知区，再按雷达与地图的吻合度做小范围微调。"""
        with self.lock:grid=self.grid;meta=self.map_meta
        pts=self._scan_in_base() if (grid is not None and self.mode=='navigation') else None
        like=self._likelihood() if pts is not None else None
        pose={k:float(p[k]) for k in ('x','y','yaw')}
        if self.mode!='navigation' or grid is None or meta is None:
            return {'applied':False,'free_shift_m':0.0,'shift_m':0.0,'shift_deg':0.0,'score':None,'samples':0,'reason':'no_map','pose':pose}
        return core_snap_pose(meta,grid,pts,pose,like=like)

    def localize(self,p):
        if self.mode!='navigation':raise ConsoleError('NOT_NAVIGATING','请先加载巡航地图')
        if self.mission['state'] in ('running','accepting','pausing','paused','stopping'):raise ConsoleError('MISSION_ACTIVE','请先停止巡航')
        self.localization_epoch+=1
        msg=PoseWithCovarianceStamped();pose=self.pose_msg(p);msg.header=pose.header;msg.pose.pose=pose.pose
        msg.pose.covariance[0]=.25;msg.pose.covariance[7]=.25;msg.pose.covariance[35]=.0685
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
        self.amcl=None
        for _ in range(90):
            if ticket!=self.localization_epoch or self.mode!='navigation':return
            if self.global_localizer.service_is_ready():break
            await asyncio.sleep(.5)
        else:raise ConsoleError('NAV_NOT_READY','定位服务未就绪')
        await ros_future(self.global_localizer.call_async(Empty.Request()))
        for _ in range(60):
            if ticket!=self.localization_epoch or self.mode!='navigation':return
            if self.nomotion.service_is_ready():await ros_future(self.nomotion.call_async(Empty.Request()),3)
            await asyncio.sleep(.5)
            if self.localized():return
        self.error='自动定位未收敛，可使用人工定位'

    def publish_speed(self):
        msg=SpeedLimit();msg.header.stamp=self.get_clock().now().to_msg();msg.percentage=False;msg.speed_limit=self.speed
        self.speed_pub.publish(msg)

    async def preview(self,points,mode):
        if self.mode!='navigation':raise ConsoleError('NOT_NAVIGATING','请先加载巡航地图')
        if not self.localized():raise ConsoleError('NOT_LOCALIZED','请先完成定位')
        with self.lock:points=validate_waypoints(points,self.map_meta,self.grid,mode)
        self.plan=[]
        if not self.planner.server_is_ready():raise ConsoleError('NAV_NOT_READY','路径规划器未就绪')
        output=[];route=points+([points[0]] if mode=='loop' else [])
        for i,p in enumerate(route):
            req=ComputePathToPose.Goal();req.goal=self.pose_msg(p);req.planner_id='';req.use_start=i>0
            if i>0:req.start=self.pose_msg(route[i-1])
            handle=await ros_future(self.planner.send_goal_async(req))
            if not handle.accepted:raise ConsoleError('PLAN_REJECTED','路径规划被拒绝')
            try:result=await ros_future(handle.get_result_async(),20)
            except Exception:
                handle.cancel_goal_async();raise
            if result.status!=4 or not result.result.path.poses:raise ConsoleError('NO_PATH',f'无法到达航点 {i+1}')
            output.extend(position(v.pose) for v in result.result.path.poses)
        with self.lock:self.plan=output[::max(1,len(output)//1500)]
        return {'path':self.plan,'point_count':len(points)}

    def start_mission(self,points,mode):
        with self.lock:
            if self.mission['state'] in ('running','accepting','pausing','paused','stopping'):raise ConsoleError('MISSION_ACTIVE','已有巡航任务')
            if not self.localized():raise ConsoleError('NOT_LOCALIZED','请先完成定位')
            if not self.nav.server_is_ready():raise ConsoleError('NAV_NOT_READY','导航服务未就绪')
            points=validate_waypoints(points,self.map_meta,self.grid,mode)
            self.epoch+=1
            self.mission={'state':'accepting','index':0,'cycle':0,'points':points,'mode':mode,'distance_remaining':None}
            self.error=None;self.publish_speed();self._send_nav(self.epoch)
            return dict(self.mission)

    def _send_nav(self,ticket):
        if ticket!=self.epoch:return
        m=self.mission;req=NavigateToPose.Goal();req.pose=self.pose_msg(m['points'][m['index']])
        def feedback(msg):
            if ticket==self.epoch:
                with self.lock:self.mission['distance_remaining']=msg.feedback.distance_remaining
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

    def _finished(self,f,ticket):
        with self.lock:
            if ticket!=self.epoch:return
            self.goal_handle=None
            try:status=f.result().status
            except Exception:status=6
            if status!=4:
                self.mission['state']='failed';self.error='导航未完成';return
            self.mission['index']+=1
            if self.mission['index']>=len(self.mission['points']):
                if self.mission['mode']=='loop':self.mission['index']=0;self.mission['cycle']+=1
                else:self.mission['state']='completed';return
            self._send_nav(ticket)

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
            if self.mission['state']!='paused':raise ConsoleError('NOT_PAUSED','任务未暂停')
            if not self.localized():raise ConsoleError('NOT_LOCALIZED','定位未就绪')
            if not self.nav.server_is_ready():raise ConsoleError('NAV_NOT_READY','导航服务未就绪')
            self.epoch+=1;self.mission['state']='accepting';self._send_nav(self.epoch)

    def snapshot(self):
        now=time.time()
        def hz(ts):return round((len(ts)-1)/(ts[-1]-ts[0]),1) if len(ts)>1 and ts[-1]>ts[0] else None
        def age(t):return round(now-t,2) if t else None
        with self.lock:
            return {'mode':self.mode,'uptime_s':int(now-self.started_at),'map':self.map_meta,
                    'pose':self.pose if now-self.pose_at<2 else None,'scan_points':self.scan_points,'path':self.plan,
                    'navigation':{'ready':self.nav.server_is_ready(),'planner_ready':self.planner.server_is_ready()},
                    'localization':{'ready':self.localized(),'match':self.match,'amcl_received':self.amcl is not None,'match_age_s':age(self.match_at),'covariance':{'x_m2':self.amcl['covariance'][0],'y_m2':self.amcl['covariance'][7],'yaw_rad2':self.amcl['covariance'][35]} if self.amcl else None},
                    'velocity':self.odom if now-self.odom_at<2 else None,
                    'imu':self.imu if now-self.imu_at<2 else None,'imu_history':list(self.imu_history),
                    'lidar':{'state':'online' if now-self.scan_at<2 else 'offline','hz':hz(self.scan_times) if now-self.scan_at<2 else None,'age_s':age(self.scan_at)},
                    'camera':{'state':'online' if now-self.camera_at<3 else 'offline','size':self.camera_size,'fps':hz(self.camera_times) if now-self.camera_at<3 else None,'age_s':age(self.camera_at)},
                    'mission':dict(self.mission),'speed_mps':self.speed,'battery_voltage':self.battery_voltage if now-self.voltage_at<5 else None,
                    'battery':{'state':'online' if now-self.voltage_at<5 or now-self.bms_at<5 else 'offline','voltage_v':self.battery_voltage if now-self.voltage_at<5 else None,'age_s':age(self.voltage_at),'percentage':self.bms['percentage'] if self.bms and now-self.bms_at<5 else None,'charging':self.charging if now-self.charging_at<5 else None,'charging_current_a':self.charging_current if now-self.current_at<5 else None,'bms':self.bms if now-self.bms_at<5 else None},
                    'chassis':{'state':'online' if now-self.raw_odom_at<2 else 'offline','model':'mini_akm','drive_type':'ackermann','age_s':age(self.raw_odom_at),'odometry':self.raw_odom if now-self.raw_odom_at<2 else None,'commanded_velocity':self.commanded if now-self.commanded_at<2 else None,'command_age_s':age(self.commanded_at)},'error':self.error}

    def close(self):
        self.spin_executor.shutdown(timeout_sec=3);self.destroy_node();rclpy.shutdown()
