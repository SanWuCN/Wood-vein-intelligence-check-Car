import asyncio
import os
import signal
import subprocess
import time
from pathlib import Path
import yaml
from core import ConsoleError


class Processes:
    def __init__(self,root,config):
        self.root=Path(root);self.config=config;self.jobs={};self.logs=self.root/'runtime'/'logs';self.logs.mkdir(parents=True,exist_ok=True)

    def spawn(self,name,args,env=None):
        old=self.jobs.get(name)
        if old and old.poll() is None:return old
        path=self.logs/(name+'.log')
        if path.exists() and path.stat().st_size>10_000_000:path.replace(path.with_suffix('.previous.log'))
        log=path.open('ab')
        clean_env={k:v for k,v in os.environ.items() if not (k.startswith('QT_') and '/cv2/' in v)}
        try:p=subprocess.Popen(args,env={**clean_env,**(env or {})},stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
        finally:log.close()
        self.jobs[name]=p;return p

    async def stop(self,name):
        p=self.jobs.pop(name,None)
        if not p:return
        # Signal the process group even when a launch parent has exited.
        for sig,wait in [(signal.SIGINT,8),(signal.SIGTERM,3),(signal.SIGKILL,1)]:
            try:os.killpg(p.pid,sig)
            except ProcessLookupError:break
            deadline=time.monotonic()+wait
            while time.monotonic()<deadline:
                p.poll()
                try:os.killpg(p.pid,0)
                except ProcessLookupError:return
                await asyncio.sleep(.1)
            p.poll()

    def existing_launches(self):
        found=[]
        for d in Path('/proc').glob('[0-9]*'):
            try:
                args=(d/'cmdline').read_bytes().decode().split('\0')
                for i,a in enumerate(args):
                    if Path(a).name=='ros2' and args[i+1:i+2]==['launch']:
                        package=args[i+2] if len(args)>i+2 else ''
                        launch=args[i+3] if len(args)>i+3 else ''
                        if (package,launch) in [('slam_gmapping','slam_gmapping.launch.py'),('slam_gmapping','slam_gmapping'),('wheeltec_nav2','wheeltec_nav2.launch.py'),('turn_on_wheeltec_robot','turn_on_wheeltec_robot.launch.py'),('turn_on_wheeltec_robot','wheeltec_lidar.launch.py')]:
                            found.append((int(d.name),package,launch))
            except (OSError,UnicodeError):continue
        return found

    async def stop_robot(self):
        await self.stop('standby')
        await self.stop('robot')
        # Adopt only known, user-owned ROS launch entry points, never arbitrary programs.
        existing=self.existing_launches()
        for pid,_,_ in existing:
            try:
                if Path('/proc',str(pid)).stat().st_uid != os.getuid():continue
                os.kill(pid,signal.SIGINT)
            except (ProcessLookupError,FileNotFoundError):pass
        deadline=time.monotonic()+12
        while self.existing_launches() and time.monotonic()<deadline:await asyncio.sleep(.15)
        names={'wheeltec_robot_node','lslidar_driver_node','ekf_node','slam_gmapping','amcl','controller_server','planner_server'}
        remaining=[]
        for d in Path('/proc').glob('[0-9]*'):
            try:
                if d.stat().st_uid!=os.getuid():continue
                args=(d/'cmdline').read_bytes().decode().split('\0');first=args[0]
                if Path(first).name in names or (Path(first).name=='component_container_isolated' and '__node:=nav2_container' in args):remaining.append(d.name)
            except (OSError,UnicodeError):continue
        if remaining:raise ConsoleError('PROCESS_CONFLICT','旧 ROS 进程未退出：'+','.join(remaining))

    def start_standby(self):
        if not self.existing_launches():
            self.spawn('standby',['ros2','launch','turn_on_wheeltec_robot','turn_on_wheeltec_robot.launch.py'])

    async def launch_robot(self,mode,map_yaml=None,params=None):
        await self.stop_robot()
        if mode=='mapping':
            # 默认参数（linearUpdate=1.0、particles=30）建出来的图会畸变，
            # 用显式参数文件起节点，走 20 cm 就处理一帧、粒子 60。
            params_path=self.root/'runtime'/'gmapping.yaml'
            params_path.parent.mkdir(parents=True,exist_ok=True)
            params_path.write_text(yaml.safe_dump({'slam_gmapping':{'ros__parameters':{
                'use_sim_time':False,'base_frame':'base_footprint','odom_frame':'odom_combined','map_frame':'map',
                'scan_topic':'scan','particles':60,'delta':0.05,'linearUpdate':0.2,'angularUpdate':0.2,
                'temporalUpdate':0.5,'resampleThreshold':0.5,'maxUrange':11.5,'minimumScore':30.0,
                'xmin':-20.0,'xmax':20.0,'ymin':-20.0,'ymax':20.0}}},sort_keys=False))
            args=['ros2','run','slam_gmapping','slam_gmapping','--ros-args','--params-file',str(params_path)]
        else:args=['ros2','launch','wheeltec_nav2','wheeltec_nav2.launch.py','map:='+str(map_yaml),'params:='+str(params)]
        p=self.spawn('robot',args)
        await asyncio.sleep(2)
        if p.poll() is not None:raise ConsoleError('LAUNCH_FAILED','ROS 启动失败，请查看运行日志')

    async def close(self):
        for name in list(self.jobs):await self.stop(name)
