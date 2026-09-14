"""Uses the real bridge cancellation method with in-memory ROS action fakes; no ROS node or motion."""
import asyncio,sys,threading,unittest
from pathlib import Path
from concurrent.futures import Future
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
try:
 from ros_bridge import RosBridge
except ImportError:
 RosBridge=None
from core import ConsoleError

def future(value=None,error=None):
 f=Future()
 if error:f.set_exception(error)
 else:f.set_result(value)
 return f
class Handle:
 accepted=True
 def __init__(self,fail=False):self.result=Future();self.cancelled=False;self.fail=fail
 def get_result_async(self):return self.result
 def cancel_goal_async(self):
  self.cancelled=True
  if self.fail:return future(error=RuntimeError('transport disconnected'))
  self.result.set_result(SimpleNamespace(status=5));return future(SimpleNamespace(return_code=0))
@unittest.skipIf(RosBridge is None,'Run on robot with system ROS libraries')
class StopTests(unittest.IsolatedAsyncioTestCase):
 def bridge(self,h,pending=False):
  self.zeros=[]
  return SimpleNamespace(cancel_lock=asyncio.Lock(),lock=threading.RLock(),epoch=1,goal_handle=None if pending else h,pending_goal=future(h) if pending else None,mission={'state':'running'},stop_pub=SimpleNamespace(publish=self.zeros.append),error=None)
 async def test_pause_confirms_terminal(self):
  h=Handle();b=self.bridge(h);await RosBridge.stop(b,pause=True)
  self.assertTrue(h.cancelled);self.assertEqual(b.mission['state'],'paused');self.assertEqual(len(self.zeros),5)
 async def test_late_acceptance_is_cancelled(self):
  h=Handle();b=self.bridge(h,True);await RosBridge.stop(b)
  self.assertTrue(h.cancelled);self.assertEqual(b.mission['state'],'stopped');self.assertEqual(b.epoch,2)
 async def test_failed_cancel_never_claims_paused(self):
  b=self.bridge(Handle(True))
  with self.assertRaises(ConsoleError):await RosBridge.stop(b,pause=True)
  self.assertEqual(b.mission['state'],'failed');self.assertEqual(len(self.zeros),5)
if __name__=='__main__':unittest.main()

@unittest.skipIf(RosBridge is None,'Run with ROS libraries')
class TelemetryTests(unittest.TestCase):
 def test_voltage_rejects_invalid(self):
  b=SimpleNamespace(lock=threading.RLock(),battery_voltage=None,voltage_at=0)
  for v in [float('nan'),float('inf'),-1,0]:RosBridge.on_voltage(b,SimpleNamespace(data=v))
  self.assertIsNone(b.battery_voltage)
  RosBridge.on_voltage(b,SimpleNamespace(data=10.85));self.assertEqual(b.battery_voltage,10.85)
 def test_bms_only_uses_valid_percentage(self):
  b=SimpleNamespace(lock=threading.RLock())
  msg=SimpleNamespace(percentage=float('nan'),current=float('nan'),temperature=float('nan'),capacity=float('nan'),charge=float('nan'),power_supply_status=0)
  RosBridge.on_battery(b,msg);self.assertIsNone(b.bms['percentage']);self.assertIsNone(b.bms['current_a'])
  msg.percentage=.65;RosBridge.on_battery(b,msg);self.assertEqual(b.bms['percentage'],65)
  msg.percentage=1.1;RosBridge.on_battery(b,msg);self.assertIsNone(b.bms['percentage'])
