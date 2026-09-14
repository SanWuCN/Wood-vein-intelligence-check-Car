#!/usr/bin/env python3
import argparse
import asyncio
import collections
import contextlib
import copy
import json
import os
import secrets
import socket
import time
from pathlib import Path
from urllib.parse import urlsplit
import aiohttp
from aiohttp import web
import psutil
import yaml
from core import ConsoleError,MapStore,finite,pose_arg,validate_waypoints,map_image
from processes import Processes
from uplink import Uplink

ROOT=Path(__file__).resolve().parent.parent


class Console:
    def __init__(self,root=ROOT,simulate=False,config_path=None):
        self.root=Path(root);self.simulate=simulate;self.config_path=Path(config_path or self.root/'config.local.json')
        self.config=json.loads((ROOT/'backend/config.example.json').read_text())
        if self.config_path.exists():self.config.update(json.loads(self.config_path.read_text()))
        if not self.config['control_token']:
            self.config['control_token']=secrets.token_urlsafe(32);self.save_config()
        if simulate:
            from simulator import SimBridge
            self.bridge=SimBridge(self.config)
        else:
            from ros_bridge import RosBridge
            self.bridge=RosBridge(self.config)
        self.maps=MapStore(self.root/'runtime/maps');self.processes=Processes(self.root,self.config)
        self.active_map_id=None;self.auto_task=None;self.tasks=[];self.media=None;self.command_lock=asyncio.Lock();self.idempotency=collections.OrderedDict()
        self.metrics={'cpu_percent':None,'memory_percent':None,'temperature_c':None};self.transition=None;self.last_error=None
        self.uplink=Uplink(self.config,self.snapshot);self.routes_path=self.root/'runtime/routes.json'
        self.routes=json.loads(self.routes_path.read_text()) if self.routes_path.exists() else []
        if simulate and not self.maps.list():self.maps.save('实验室地图',*self.bridge.snapshot_map())
        if not simulate:
            existing=self.processes.existing_launches()
            if any(p=='slam_gmapping' for _,p,_ in existing):self.bridge.mode='mapping'
            elif any(p=='wheeltec_nav2' for _,p,_ in existing):self.bridge.mode='navigation'
        self.app=web.Application(client_max_size=2*1024**2,middlewares=[self.middleware])
        r=self.app.router
        r.add_get('/api/battery',self.battery);r.add_get('/api/chassis',self.chassis);r.add_get('/api/health',self.health);r.add_get('/api/session',self.session);r.add_get('/api/state',self.state)
        r.add_get('/api/ws',self.websocket);r.add_get('/api/maps',self.map_list);r.add_get('/api/map',self.map_metadata);r.add_get('/api/map.png',self.map_png)
        r.add_get('/api/maps/{id}/preview.png',self.saved_preview);r.add_get('/api/maps/{id}/files/{file}',self.map_file)
        r.add_get('/api/routes',self.route_list);r.add_get('/api/config',self.get_config)
        r.add_get('/api/streams/{channel}.mjpeg',self.stream)
        r.add_post('/api/{group}/{action}',self.command)
        r.add_get('/{tail:.*}',self.static)
        self.app.on_startup.append(self.startup);self.app.on_cleanup.append(self.cleanup)

    def save_config(self):
        self.config_path.parent.mkdir(parents=True,exist_ok=True)
        temp=self.config_path.with_suffix('.tmp');temp.write_text(json.dumps(self.config,ensure_ascii=False,indent=2));temp.chmod(0o600);temp.replace(self.config_path)

    def authorized(self,req):
        return secrets.compare_digest(req.headers.get('X-Control-Token',''),self.config['control_token'])

    @web.middleware
    async def middleware(self,req,handler):
        try:
            if req.method not in ('GET','HEAD','OPTIONS'):
                origin=req.headers.get('Origin')
                if origin and urlsplit(origin).netloc!=req.host:raise ConsoleError('BAD_ORIGIN','请求来源无效',403)
                if not self.authorized(req):raise ConsoleError('CONTROL_LOCKED','控制权限未开启',403)
                if req.content_type!='application/json':raise ConsoleError('JSON_REQUIRED','请使用 JSON 请求',415)
            response=await handler(req)
            if req.path.startswith('/api/'):
                response.headers['Cache-Control']='no-store'
            response.headers['X-Content-Type-Options']='nosniff'
            return response
        except ConsoleError as e:
            return web.json_response({'ok':False,'error':{'code':e.code,'message':str(e)}},status=e.status)
        except (json.JSONDecodeError,KeyError,TypeError,ValueError) as e:
            return web.json_response({'ok':False,'error':{'code':'INVALID_REQUEST','message':str(e)}},status=422)
        except web.HTTPException:raise
        except Exception as e:
            import logging;logging.exception('Console request failed')
            return web.json_response({'ok':False,'error':{'code':'INTERNAL_ERROR','message':'操作失败，请检查服务日志'}},status=500)

    async def battery(self,req):return web.json_response(self.bridge.snapshot().get('battery',{'state':'offline'}))
    async def chassis(self,req):return web.json_response(self.bridge.snapshot().get('chassis',{'state':'offline'}))
    async def health(self,req):return web.json_response({'ok':True,'version':'1.0.0','simulated':self.simulate})
    async def session(self,req):
        local=req.remote in ('127.0.0.1','::1')
        return web.json_response({'token':self.config['control_token'] if local else None,'can_control':local,'device_id':self.config['device_id'],'max_speed_mps':self.config['max_speed_mps']})
    async def state(self,req):return web.json_response(self.snapshot())
    def snapshot(self):
        state=self.bridge.snapshot()
        state.update({'schema_version':'1.0','device_id':self.config['device_id'],'sampled_at':time.time(),'simulated':self.simulate,'metrics':dict(self.metrics),'active_map_id':self.active_map_id,'transition':self.transition,'max_speed_mps':self.config['max_speed_mps'],
                      'platform':{'state':self.uplink.state,'last_success':self.uplink.last_success,'error':self.uplink.error},
                      'streams':{'rviz':'/api/streams/rviz.mjpeg','camera':'/api/streams/camera.mjpeg','rviz_state':('online' if self.media and time.time()-self.media.rviz_at<3 else 'offline'),'rtmp':self.media.rtmp if self.media else {}},'last_error':self.last_error})
        return state

    async def websocket(self,req):
        origin=req.headers.get('Origin')
        if origin and urlsplit(origin).netloc!=req.host:raise ConsoleError('BAD_ORIGIN','请求来源无效',403)
        ws=web.WebSocketResponse(heartbeat=15,receive_timeout=45);await ws.prepare(req)
        try:
            while not ws.closed:
                await ws.send_json({'type':'state','payload':self.snapshot()})
                try:
                    msg=await asyncio.wait_for(ws.receive(),timeout=.5)
                    if msg.type in (aiohttp.WSMsgType.CLOSE,aiohttp.WSMsgType.CLOSED,aiohttp.WSMsgType.ERROR):break
                except asyncio.TimeoutError:pass
        except (ConnectionError,asyncio.CancelledError):pass
        return ws

    async def map_list(self,req):return web.json_response({'maps':self.maps.list()})
    async def map_metadata(self,req):
        if not self.bridge.map_meta:raise ConsoleError('MAP_UNAVAILABLE','地图未就绪',404)
        return web.json_response(self.bridge.map_meta)
    async def map_png(self,req):
        if not self.bridge.map_png:raise ConsoleError('MAP_UNAVAILABLE','地图未就绪',404)
        return web.Response(body=self.bridge.map_png,content_type='image/png')
    async def saved_preview(self,req):return web.FileResponse(self.maps.directory(req.match_info['id'])/'preview.png')
    async def map_file(self,req):
        name=req.match_info['file']
        if name not in ('map.yaml','map.pgm','metadata.json'):raise ConsoleError('NOT_FOUND','文件不存在',404)
        return web.FileResponse(self.maps.directory(req.match_info['id'])/name)
    async def route_list(self,req):return web.json_response({'routes':self.routes})
    async def get_config(self,req):
        if not self.authorized(req):raise ConsoleError('CONTROL_LOCKED','控制权限未开启',403)
        fields=('device_id','platform_url','rviz_rtmp_url','camera_rtmp_url','telemetry_interval_s')
        return web.json_response({**{k:self.config[k] for k in fields},'platform_token_set':bool(self.config['platform_token'])})

    async def command(self,req):
        group,action=req.match_info['group'],req.match_info['action'];body=await req.json()
        if not isinstance(body,dict):raise ConsoleError('INVALID_REQUEST','请求体必须为对象',422)
        request_id=req.headers.get('X-Request-Id','')
        fingerprint=json.dumps([group,action,body],sort_keys=True)
        if len(request_id)>120:raise ConsoleError('INVALID_REQUEST_ID','请求编号过长',422)
        if request_id and request_id in self.idempotency:
            previous,result=self.idempotency[request_id]
            if previous!=fingerprint:raise ConsoleError('IDEMPOTENCY_CONFLICT','请求编号已用于其他操作')
            return web.json_response(result)
        # Stop never waits behind a map transition or a planning request.
        if (group,action)==('control','stop'):
            await self.safe_stop();result={'ok':True,'state':'stopped'}
        else:
            async with self.command_lock:
                if request_id and request_id in self.idempotency:
                    previous,result=self.idempotency[request_id]
                    if previous!=fingerprint:raise ConsoleError('IDEMPOTENCY_CONFLICT','请求编号已用于其他操作')
                    return web.json_response(result)
                result=await self.dispatch(group,action,body)
        if request_id:
            self.idempotency[request_id]=(fingerprint,result)
            if len(self.idempotency)>256:self.idempotency.popitem(last=False)
        return web.json_response(result)

    async def dispatch(self,group,action,body):
        key=(group,action)
        if key in [('mapping','start'),('mapping','restart')]:
            if self.bridge.mode=='mapping' and action=='start':raise ConsoleError('ALREADY_MAPPING','建图已运行')
            if self.bridge.mission['state'] in ('running','accepting','paused','pausing','stopping'):raise ConsoleError('MISSION_ACTIVE','请先停止巡航')
            backup=None
            if self.bridge.mode=='mapping' and self.bridge.map_meta:
                backup=self.maps.save('自动备份 '+time.strftime('%m-%d %H:%M'),*self.bridge.snapshot_map())
            await self.switch_mode('mapping');return {'ok':True,'mode':'mapping','backup':backup}
        if key==('mapping','save'):
            item=self.maps.save(body.get('name'),*self.bridge.snapshot_map());return {'ok':True,'map':item}
        if key==('mapping','stop'):
            await self.safe_stop()
            if not self.simulate:await self.processes.stop_robot()
            self.bridge.mode='idle'
            if not self.simulate:self.processes.start_standby()
            return {'ok':True,'mode':'idle'}
        if key==('navigation','load'):
            if self.bridge.mission['state'] in ('running','accepting','paused','pausing','stopping'):raise ConsoleError('MISSION_ACTIVE','请先停止巡航')
            item,grid=self.maps.load(body.get('map_id'))
            if self.bridge.mode=='mapping' and self.bridge.map_meta:self.maps.save('切换前备份 '+time.strftime('%m-%d %H:%M'),*self.bridge.snapshot_map())
            await self.switch_mode('navigation',item)
            self.active_map_id=item['id']
            if self.simulate:
                self.bridge.map_meta=item['map'];self.bridge.grid=grid;self.bridge.map_png=map_image(grid)
            self.start_auto_localize()
            return {'ok':True,'map':item,'localization':'pending'}
        if key==('navigation','auto-localize'):
            if self.bridge.mode!='navigation':raise ConsoleError('NOT_NAVIGATING','请先加载巡航地图')
            if self.bridge.mission['state'] in ('running','accepting','paused','pausing','stopping'):raise ConsoleError('MISSION_ACTIVE','请先停止巡航')
            self.start_auto_localize();return {'ok':True,'state':'localizing'}
        if key==('navigation','refine'):
            enabled=body.get('enabled')
            if not isinstance(enabled,bool):raise ConsoleError('INVALID_ARGUMENT','enabled 必须为布尔值',422)
            return {'ok':True,'refine':self.bridge.set_idle_refine(enabled)}

        if key==('navigation','localize'):
            p=pose_arg(body.get('pose'))
            # 吸附微调：先移出障碍/未知区，再按雷达与地图的吻合度做小范围微调
            snap=None
            if not self.simulate and body.get('snap',True) is not False:
                # 放到线程里算，避免长时间占用事件循环影响状态推流
                try:snap=await asyncio.to_thread(self.bridge.snap_pose,p)
                except Exception:snap=None
                if snap and snap.get('pose'):p=snap['pose']
            validate_waypoints([p],self.bridge.map_meta,self.bridge.grid,'single')
            self.bridge.localize(p)
            if not self.simulate:
                if self.auto_task:self.auto_task.cancel()
                async def refine():
                    try:await self.bridge.refine_localization()
                    except asyncio.CancelledError:pass
                    except Exception as e:self.bridge.error=str(e)
                self.auto_task=asyncio.create_task(refine())
            return {'ok':True,'state':'localizing','pose':p,'snap':snap}
        if key==('navigation','preview'):
            result=await self.bridge.preview(body.get('points'),body.get('mode','multi'));return {'ok':True,**result}
        if key==('navigation','start'):
            if body.get('map_id')!=self.active_map_id or not self.active_map_id:raise ConsoleError('MAP_MISMATCH','任务地图与已加载地图不一致')
            speed=finite(body.get('speed_mps'), 'speed_mps',.05,self.config['max_speed_mps']);self.bridge.speed=speed
            result=self.bridge.start_mission(body.get('points'),body.get('mode','multi'));return {'ok':True,'mission':result}
        if key==('navigation','pause'):
            if self.bridge.mission['state'] not in ('running','accepting'):raise ConsoleError('NOT_RUNNING','巡航未运行')
            await self.safe_stop(pause=True);return {'ok':True,'state':'paused'}
        if key==('navigation','resume'):
            self.bridge.resume();return {'ok':True,'state':'accepting'}
        if key==('navigation','speed'):
            self.bridge.speed=finite(body.get('speed_mps'),'speed_mps',.05,self.config['max_speed_mps']);self.bridge.publish_speed()
            return {'ok':True,'speed_mps':self.bridge.speed}
        if key==('routes','save'):
            import uuid
            item,grid=self.maps.load(body.get('map_id'))
            points=validate_waypoints(body.get('points'),item['map'],grid,body.get('mode','multi'))
            name=body.get('name','')
            if not isinstance(name,str) or not 1<=len(name.strip())<=40:raise ConsoleError('INVALID_NAME','路线名称应为 1–40 字',422)
            route={'id':uuid.uuid4().hex,'name':name.strip(),'map_id':item['id'],'mode':body.get('mode','multi'),'points':points,'speed_mps':finite(body.get('speed_mps'),'speed_mps',.05,self.config['max_speed_mps'])}
            self.routes.append(route);self.routes=self.routes[-200:]
            temp=self.routes_path.with_suffix('.tmp');temp.write_text(json.dumps(self.routes,ensure_ascii=False));temp.replace(self.routes_path)
            return {'ok':True,'route':route}
        if key==('settings','save'):
            for k in ('platform_url','rviz_rtmp_url','camera_rtmp_url'):
                if k not in body:continue
                url=body[k]
                if not isinstance(url,str) or len(url)>2000:raise ConsoleError('INVALID_URL','地址格式错误',422)
                allowed=('http','https') if k=='platform_url' else ('rtmp','rtmps')
                if url and (urlsplit(url).scheme not in allowed or not urlsplit(url).hostname):raise ConsoleError('INVALID_URL','地址协议无效',422)
            updated={k:body[k] for k in ('platform_url','rviz_rtmp_url','camera_rtmp_url') if k in body}
            if 'platform_token' in body:
                if not isinstance(body['platform_token'],str) or len(body['platform_token'])>2000:raise ConsoleError('INVALID_TOKEN','令牌格式错误',422)
                updated['platform_token']=body['platform_token']
            self.config.update(updated);self.save_config()
            if self.media:await self.media.reconfigure_rtmp()
            return {'ok':True}
        raise ConsoleError('NOT_FOUND','接口不存在',404)

    def start_auto_localize(self):
        if self.simulate:return
        if self.auto_task:self.auto_task.cancel()
        async def perform():
            try:await self.bridge.auto_localize()
            except asyncio.CancelledError:pass
            except Exception as e:self.bridge.error=str(e)
        self.auto_task=asyncio.create_task(perform())

    async def safe_stop(self,pause=False):
        try:await self.bridge.stop(pause=pause)
        except ConsoleError:
            if not self.simulate:await self.processes.stop_robot()
            self.bridge.mode='idle';self.bridge.mission['state']='failed'
            self.last_error='导航已关闭，请重新加载地图'
            if pause:raise ConsoleError('NAV_SHUTDOWN','取消未确认，导航已关闭；请重新加载地图',503)

    async def watchdog(self):
        lost_since=None
        while True:
            if not self.simulate and self.bridge.mission['state'] in ('running','accepting'):
                state=self.bridge.snapshot()
                if not state['localization']['ready'] or state['lidar']['state']!='online' or not state['velocity']:
                    lost_since=lost_since or time.monotonic()
                    if time.monotonic()-lost_since>2:
                        await self.safe_stop()
                        self.bridge.mission['state']='failed';self.bridge.error='定位或传感器数据中断，任务已停止'
                        lost_since=None
                else:lost_since=None
            else:lost_since=None
            await asyncio.sleep(.25)

    async def switch_mode(self,mode,item=None):
        self.transition=mode;self.last_error=None
        try:
            await self.safe_stop()
            if not self.simulate:
                params=None
                if mode=='navigation':
                    source=Path(self.config['workspace'])/'install/wheeltec_nav2/share/wheeltec_nav2/param/wheeltec_params/param_mini_akm.yaml'
                    cfg=yaml.safe_load(source.read_text());amcl=cfg['amcl']['ros__parameters'];amcl['set_initial_pose']=False
                    params=self.root/'runtime/navigation.yaml';params.write_text(yaml.safe_dump(cfg,sort_keys=False))
                await self.processes.stop_robot()
                self.bridge.reset_localization();self.bridge.mode='idle'
                await self.processes.launch_robot(mode,self.maps.directory(item['id'])/'map.yaml' if item else None,params)
            else:self.bridge.reset_localization()
            self.bridge.mode=mode;self.bridge.started_at=time.time()
            if mode=='mapping':self.active_map_id=None
        except Exception as e:self.last_error=str(e);self.bridge.mode='idle';raise
        finally:self.transition=None

    async def stream(self,req):
        ch=req.match_info['channel']
        if ch not in ('camera','rviz'):raise ConsoleError('NOT_FOUND','视频通道不存在',404)
        if not self.media:raise ConsoleError('STREAM_UNAVAILABLE','视频未就绪',503)
        response=web.StreamResponse(headers={'Content-Type':'multipart/x-mixed-replace; boundary=frame','Cache-Control':'no-store','X-Accel-Buffering':'no'})
        await response.prepare(req);last=0
        try:
            while True:
                frame,stamp=self.media.frame(ch)
                if frame and stamp!=last and time.time()-stamp<3:
                    last=stamp
                    await asyncio.wait_for(response.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(frame)).encode()+b'\r\n\r\n'+frame+b'\r\n'),timeout=5)
                await asyncio.sleep(1/self.config['video_fps'])
        except (ConnectionError,asyncio.CancelledError,asyncio.TimeoutError):pass
        return response

    async def static(self,req):
        dist=self.root/'dist';tail=req.match_info['tail'] or 'index.html';p=(dist/tail).resolve()
        if not p.is_relative_to(dist.resolve()):raise web.HTTPNotFound()
        if not p.is_file():p=dist/'index.html'
        if not p.exists():return web.Response(text='Frontend build unavailable',status=503)
        return web.FileResponse(p)

    async def metrics_loop(self):
        psutil.cpu_percent()
        while True:
            temp=[]
            for p in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
                try:
                    v=float(p.read_text())/1000
                    if 0<v<150:temp.append(v)
                except (ValueError,OSError):pass
            self.metrics={'cpu_percent':psutil.cpu_percent(),'memory_percent':psutil.virtual_memory().percent,'temperature_c':round(max(temp),1) if temp else None}
            await asyncio.sleep(1)

    async def startup(self,app):
        self.tasks.append(asyncio.create_task(self.watchdog()));self.tasks.append(asyncio.create_task(self.metrics_loop()));self.tasks.append(asyncio.create_task(self.uplink.run()))
        if not self.simulate:
            if self.bridge.mode=='idle':self.processes.start_standby()
            from media import Media
            self.media=Media(self.config,self.processes,self.bridge,self.root)
            async def start_media():
                try:
                    await self.media.initialize()
                    self.tasks.append(asyncio.create_task(self.media.capture_loop()));self.tasks.append(asyncio.create_task(self.media.monitor()))
                    await self.media.reconfigure_rtmp()
                except Exception as e:self.last_error='RViz: '+str(e)
            self.tasks.append(asyncio.create_task(start_media()))
            if self.config['camera_auto_start']:
                self.processes.spawn('camera',['ros2','launch','astra_camera','astra.launch.xml','enable_d2c_viewer:=false','enable_point_cloud:=false','enable_depth:=false','enable_ir:=false'])

    async def cleanup(self,app):
        if self.auto_task:self.auto_task.cancel()
        self.uplink.running=False
        if self.media:self.media.running=False
        for task in self.tasks:task.cancel()
        for task in self.tasks:
            with contextlib.suppress(asyncio.CancelledError):await task
        with contextlib.suppress(Exception):await self.safe_stop()
        await self.processes.close();self.bridge.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--simulate',action='store_true');parser.add_argument('--port',type=int);parser.add_argument('--config');args=parser.parse_args()
    console=Console(simulate=args.simulate,config_path=args.config)
    if args.port:console.config['port']=args.port
    web.run_app(console.app,host=console.config['host'],port=console.config['port'],access_log=None,shutdown_timeout=3)
