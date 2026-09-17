import sys, tempfile, unittest, math, json, asyncio
from unittest.mock import AsyncMock,patch
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
import numpy as np
from aiohttp.test_utils import TestClient,TestServer
from core import *
from app import Console

class GeometryTests(unittest.TestCase):
 def test_rotated_map_roundtrip(self):
  for a in [0,.3,-1.9,math.pi]:
   m={'origin':{'x':-4.2,'y':8.4,'yaw':a},'resolution':.05,'width':20,'height':30}
   for x,y in [(0,0),(9,17),(19,29)]:
    wx,wy=cell_to_world(m,x+.5,y+.5)
    self.assertEqual(world_to_cell(m,wx,wy),(x,y))
 def test_finite_and_bounds(self):
  for v in [float('nan'),float('inf'),True,'0.1',0,.36]:
   with self.assertRaises(ConsoleError):finite(v,'speed',.05,.35)
 def test_map_roundtrip_and_unknown(self):
  with tempfile.TemporaryDirectory() as d:
   store=MapStore(d);grid=np.array([[-1,0],[100,0]],dtype=np.int16)
   m={'origin':{'x':-1,'y':-1,'yaw':.3},'resolution':.1,'width':2,'height':2,'frame_id':'map'}
   item=store.save('场景',m,grid);loaded,g=store.load(item['id'])
   self.assertTrue(np.array_equal(grid,g));self.assertEqual(loaded['map'],m)
   for v in ['../outside','/tmp','x/y',None]:
    with self.assertRaises(ConsoleError):store.directory(v)
   for xy in [(0,0),(0,1),(2,2)]:
    x,y=cell_to_world(m,xy[0]+.5,xy[1]+.5)
    with self.assertRaises(ConsoleError):validate_waypoints([{'x':x,'y':y,'yaw':0}],m,g,'single')

class APITests(unittest.IsolatedAsyncioTestCase):
 async def asyncSetUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.console=Console(root=self.tmp.name,simulate=True)
  self.client=TestClient(TestServer(self.console.app));await self.client.start_server()
  self.headers={'X-Control-Token':self.console.config['control_token']}
 async def asyncTearDown(self):
  await self.client.close();self.tmp.cleanup()
 async def post(self,path,data,headers=None):
  r=await self.client.post('/api/'+path,json=data,headers=headers if headers is not None else self.headers)
  return r.status,await r.json()
 async def test_environment_reset_preserves_maps_and_routes(self):
  before=self.console.maps.list();self.console.routes=[{'id':'keep'}]
  self.console.bridge.error='old failure';self.console.active_map_id=before[0]['id']
  status,result=await self.post('control/reset',{})
  self.assertEqual(status,200);self.assertEqual(self.console.bridge.mode,'idle')
  self.assertEqual(self.console.bridge.mission['state'],'idle');self.assertIsNone(self.console.active_map_id)
  self.assertIsNone(self.console.bridge.error);self.assertEqual(self.console.routes,[{'id':'keep'}])
  self.assertTrue(set(x['id'] for x in before).issubset(x['id'] for x in self.console.maps.list()))
  self.assertIsNotNone(result['backup'])
 async def test_auth_and_origin(self):
  self.assertEqual((await self.post('navigation/speed',{'speed_mps':.2},{}))[0],403)
  self.assertEqual((await self.post('navigation/speed',{'speed_mps':.2},{**self.headers,'Origin':'http://evil.invalid'}))[0],403)
 async def test_rejected_speed_does_not_change(self):
  old=self.console.bridge.speed
  for v in [-.01,.21,.35,True,'0.2']:
   self.assertEqual((await self.post('navigation/speed',{'speed_mps':v}))[0],422)
  self.assertEqual(self.console.bridge.speed,old)
 async def test_cruise_speed_defaults_and_boundaries(self):
  self.assertEqual(self.console.bridge.speed,.05)
  self.assertEqual(self.console.config['max_speed_mps'],.2)
  for speed in [0,.01,.05,.2]:
   status,body=await self.post('navigation/speed',{'speed_mps':speed})
   self.assertEqual(status,200);self.assertEqual(body['speed_mps'],speed)
   self.assertEqual(self.console.bridge.speed,speed)
 async def test_zero_speed_pauses_and_positive_speed_does_not_resume(self):
  b=self.console.bridge;b.mission.update(state='running',points=[{'x':0,'y':0}])
  with patch.object(b,'publish_speed') as publish:
   status,_=await self.post('navigation/speed',{'speed_mps':0})
   self.assertEqual(status,200);self.assertEqual(b.mission['state'],'paused');publish.assert_not_called()
   status,result=await self.post('navigation/resume',{})
   self.assertEqual(status,409);self.assertEqual(result['error']['code'],'ZERO_SPEED')
   self.assertEqual(b.mission['state'],'paused')
   status,_=await self.post('navigation/speed',{'speed_mps':.01})
   self.assertEqual(status,200);self.assertEqual(b.mission['state'],'paused');publish.assert_called_once()
   status,_=await self.post('navigation/resume',{'speed_mps':.05})
   self.assertEqual(status,200);self.assertEqual(b.speed,.05);self.assertEqual(b.mission['state'],'running')
 async def test_zero_speed_cancellation_failure_is_not_success(self):
  b=self.console.bridge;b.mission['state']='running'
  with patch.object(b,'stop',AsyncMock(side_effect=ConsoleError('CANCEL_UNCONFIRMED','failed'))),patch.object(b,'publish_speed') as publish:
   status,_=await self.post('navigation/speed',{'speed_mps':0})
   self.assertEqual(status,503);self.assertEqual(b.mission['state'],'failed');publish.assert_not_called()
 async def test_zero_speed_never_starts_a_goal(self):
  b=self.console.bridge;self.console.active_map_id='test'
  with patch.object(b,'start_mission',AsyncMock()) as start:
   for speed in [0,-.01,.21]:
    status,_=await self.post('navigation/start',{'map_id':'test','points':[{'x':0,'y':0}],'speed_mps':speed})
    self.assertIn(status,(409,422))
   start.assert_not_called()
 async def test_routes_enforce_new_speed_range(self):
  m=self.console.maps.list()[0]
  for speed,expected in [(0,200),(.01,200),(.2,200),(.21,422)]:
   status,_=await self.post('routes/save',{'map_id':m['id'],'name':'speed','points':[{'x':0,'y':0}],'mode':'single','speed_mps':speed})
   self.assertEqual(status,expected)
 async def test_old_config_and_saved_route_use_new_limits(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);(root/'runtime').mkdir()
   (root/'config.local.json').write_text(json.dumps({'max_speed_mps':.35,'default_speed_mps':.2,'control_token':'existing'}))
   (root/'runtime/routes.json').write_text(json.dumps([{'points':[],'speed_mps':.35},{'points':[],'speed_mps':0}]))
   c=Console(root=root,simulate=True)
   self.assertEqual(c.config['max_speed_mps'],.2);self.assertEqual(c.bridge.speed,.05)
   self.assertEqual([r['speed_mps'] for r in c.routes],[.2,0])
   saved=json.loads((root/'config.local.json').read_text())
   self.assertEqual(saved['max_speed_mps'],.2);self.assertEqual(saved['default_speed_mps'],.05)
   self.assertEqual(saved['control_token'],'existing')
 async def test_idempotent_save(self):
  h={**self.headers,'X-Request-Id':'unique-save'}
  a=await self.post('mapping/save',{'name':'接口测试'},h);b=await self.post('mapping/save',{'name':'接口测试'},h)
  self.assertEqual(a,b);self.assertEqual(len(self.console.maps.list()),2)
  self.assertEqual((await self.post('mapping/save',{'name':'另一个'},h))[0],409)
 async def test_map_and_route_validation(self):
  status,body=await self.post('navigation/start',{'map_id':'wrong','points':[{'x':0,'y':0,'yaw':0}],'mode':'single','speed_mps':.2})
  self.assertEqual(body['error']['code'],'MAP_MISMATCH')
  m=self.console.maps.list()[0]
  status,body=await self.post('routes/save',{'map_id':m['id'],'name':'测试路线','points':[{'x':0,'y':0,'yaw':0}],'mode':'loop','speed_mps':.2})
  self.assertEqual(status,422)
 async def test_map_load_does_not_start_motion(self):
  m=self.console.maps.list()[0]
  self.assertEqual((await self.post('navigation/load',{'map_id':m['id']}))[0],200)
  self.assertEqual(self.console.bridge.mode,'navigation')
  self.assertNotIn(self.console.bridge.mission['state'],['running','accepting'])
  self.assertFalse(self.console.bridge.localized())
 async def test_websocket_is_real_schema(self):
  ws=await self.client.ws_connect('/api/ws');message=await ws.receive_json(timeout=2)
  self.assertEqual(message['type'],'state');self.assertTrue(message['payload']['simulated'])
  self.assertIn('metrics',message['payload']);await ws.close()

if __name__=='__main__':unittest.main()
