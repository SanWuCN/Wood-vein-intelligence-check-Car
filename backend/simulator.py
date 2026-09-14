"""Explicit desktop QA simulator; never selected automatically on a robot."""
import math
import threading
import time
import numpy as np
from core import bounds,map_image,validate_waypoints


class SimBridge:
    def __init__(self,config):
        self.config=config;self.lock=threading.RLock();self.mode='mapping';self.started_at=time.time();self.error=None
        self.grid=np.full((360,460),-1,np.int16);self.grid[40:320,40:420]=0
        self.grid[40:43,40:420]=100;self.grid[317:320,40:420]=100;self.grid[40:320,40:43]=100;self.grid[40:320,417:420]=100
        self.grid[40:145,170:174]=100;self.grid[180:320,170:174]=100
        self.grid[185:190,40:115]=100;self.grid[185:190,270:420]=100
        for x,y in [(235,105),(290,105),(235,255),(290,255)]:self.grid[y:y+12,x:x+12]=100
        self.map_meta={'width':460,'height':360,'resolution':.05,'origin':{'x':-11.5,'y':-9.,'yaw':0.},'bounds':bounds(self.grid),'frame_id':'map','revision':1}
        self.map_png=map_image(self.grid);self.pose={'x':0.,'y':0.,'yaw':.3};self.camera_jpeg=None;self.camera_at=0
        self.speed=config['default_speed_mps'];self.plan=[];self.ready=False
        self.mission={'state':'idle','points':[],'mode':'multi','index':0,'cycle':0,'distance_remaining':None}
    def snapshot_map(self):return dict(self.map_meta),self.grid.copy(),dict(self.pose)
    def reset_localization(self):self.ready=False
    def localize(self,p):self.pose=p;self.ready=True
    def localized(self):return self.ready
    def publish_speed(self):pass
    async def preview(self,points,mode):
        ps=validate_waypoints(points,self.map_meta,self.grid,mode);self.plan=[self.pose]+ps
        return {'path':self.plan,'point_count':len(ps)}
    def start_mission(self,points,mode):
        ps=validate_waypoints(points,self.map_meta,self.grid,mode)
        self.mission={'state':'running','points':ps,'mode':mode,'index':0,'cycle':0,'distance_remaining':2.4};return self.mission
    async def stop(self,pause=False):self.mission['state']='paused' if pause else 'stopped'
    def resume(self):self.mission['state']='running'
    def snapshot(self):
        t=time.time();s=math.sin(t)*.12
        return {'mode':self.mode,'uptime_s':int(t-self.started_at),'map':self.map_meta,'pose':self.pose,'path':self.plan,'scan_points':[],
                'navigation':{'ready':self.mode=='navigation','planner_ready':self.mode=='navigation'},'localization':{'ready':self.ready,'match':.91 if self.ready else None,'amcl_received':self.ready},
                'velocity':{'linear_mps':0.,'angular_rps':0.},'imu':{'roll_deg':s,'pitch_deg':-.23,'yaw_deg':17.2,'angular_velocity':{'x':0.,'y':0.,'z':0.},'linear_acceleration':{'x':0.,'y':0.,'z':9.8}},
                'imu_history':[[t-i,math.sin(i/7)*.3,math.cos(i/9)*.3,17.2] for i in range(80,0,-1)],
                'lidar':{'state':'online','hz':10.,'age_s':.04},'camera':{'state':'offline','fps':None,'age_s':None},'mission':dict(self.mission),'speed_mps':self.speed,'battery_voltage':None,'error':self.error}
    def close(self):pass
