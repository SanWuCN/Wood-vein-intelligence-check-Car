import asyncio,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
from aiohttp import web
from aiohttp.test_utils import TestServer
from uplink import Uplink
class UplinkTests(unittest.IsolatedAsyncioTestCase):
 async def test_registration_envelope_and_ack(self):
  received=[];done=asyncio.Event()
  async def register(r):
   received.append((r.path,await r.json(),dict(r.headers)));return web.json_response({'ok':True})
  async def telemetry(r):
   b=await r.json();received.append((r.path,b,dict(r.headers)));done.set()
   return web.json_response({'accepted':[b['events'][0]['messageId']],'duplicated':[],'rejected':[]})
  app=web.Application();app.router.add_post('/api/devices/register',register);app.router.add_post('/api/device-events/batch',telemetry)
  async with TestServer(app) as server:
   c={'platform_url':str(server.make_url('')).rstrip('/'),'platform_token':'test-token','device_id':'test-car','telemetry_interval_s':.05}
   u=Uplink(c,lambda:{'metrics':{'temperature_c':61},'scan_points':[[1,2]],'path':[],'imu_history':[]})
   task=asyncio.create_task(u.run())
   try:
    await asyncio.wait_for(done.wait(),3);await asyncio.sleep(.02)
    self.assertEqual(u.state,'online');self.assertEqual(received[0][2]['X-Device-Token'],'test-token')
    e=received[1][1]['events'][0];self.assertEqual(e['type'],'device.telemetry');self.assertNotIn('scan_points',e['payload']);self.assertEqual(e['payload']['metrics']['temperature_c'],61)
   finally:u.running=False;task.cancel()
   try:await task
   except asyncio.CancelledError:pass
if __name__=='__main__':unittest.main()
