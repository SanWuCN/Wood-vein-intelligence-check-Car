"""Uses the real bridge cancellation method with in-memory ROS action fakes; no ROS node or motion."""
import asyncio,sys,threading,unittest
from unittest.mock import Mock, patch
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
 async def test_zero_speed_never_publishes_nav2_unlimited_message(self):
  b=SimpleNamespace(speed=0,speed_pub=Mock())
  RosBridge.publish_speed(b);b.speed_pub.publish.assert_not_called()
 async def test_zero_speed_blocks_start_and_resume(self):
  b=SimpleNamespace(speed=0,lock=threading.RLock())
  with self.assertRaises(ConsoleError):await RosBridge.start_mission(b,[],'single')
  with self.assertRaises(ConsoleError):RosBridge.resume(b)
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

@unittest.skipIf(RosBridge is None,'Run with ROS libraries')
class NavigationResultTests(unittest.TestCase):
 def bridge(self):
  return SimpleNamespace(lock=threading.RLock(),epoch=3,goal_handle=object(),pending_goal=object(),
   _skipping=False,mission={'state':'running','skipped':0},map_meta=None,grid=None,
   pose=None,active_clearance=lambda:.1,plan=[1],match=.99,error=None)
 def test_cancelled_result_produces_reason_without_crashing_executor(self):
  b=self.bridge()
  with patch('ros_bridge.start_conflict',return_value=False):
   RosBridge._finished(b,future(SimpleNamespace(status=5)),3)
  self.assertEqual(b.mission['state'],'failed')
  self.assertEqual(b.mission['stop_reason'],'任务被取消')
  self.assertIsNone(b.pending_goal)
 def test_stuck_reason_survives_cancel_result(self):
  b=self.bridge();b.mission.update(state='failed',abort_reason='stuck',stop_reason='No progress for 24 seconds')
  RosBridge._finished(b,future(SimpleNamespace(status=5)),3)
  self.assertEqual(b.mission['abort_reason'],'stuck')
  self.assertEqual(b.mission['stop_reason'],'No progress for 24 seconds')
  self.assertIsNone(b.pending_goal)
 def test_old_result_does_not_overwrite_new_mission(self):
  b=self.bridge();RosBridge._finished(b,future(SimpleNamespace(status=6)),2)
  self.assertEqual(b.mission['state'],'running')
 def test_executor_failure_stops_and_requests_service_restart(self):
  b=SimpleNamespace(spin_executor=SimpleNamespace(spin=Mock(side_effect=RuntimeError('callback failed'))),
                    _publish_stop_now=Mock())
  with patch('ros_bridge.os._exit') as exit_process, self.assertLogs(level='ERROR'):
   RosBridge._spin(b)
  b._publish_stop_now.assert_called_once()
  exit_process.assert_called_once_with(1)
