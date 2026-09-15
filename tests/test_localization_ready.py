"""Initialization ordering tests: ROS clients are mocks, no node or actuator."""
import asyncio
import sys
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
try:
 from ros_bridge import RosBridge
except ImportError:
 RosBridge=None

def done(v):
 f=Future();f.set_result(v);return f

@unittest.skipIf(RosBridge is None,'Run on robot with system ROS libraries')
class ReadinessTests(unittest.IsolatedAsyncioTestCase):
 def bridge(self):
  return NS(mode='navigation',mission={'state':'idle'},localization_epoch=0,map_meta={'width':10},amcl=None,error=None,
   amcl_lifecycle=Mock(),global_localizer=Mock(),nomotion=Mock(),localized=Mock(return_value=True))
 async def test_no_global_request_until_active(self):
  b=self.bridge();b.amcl_lifecycle.call_async.side_effect=[done(NS(current_state=NS(id=2))),done(NS(current_state=NS(id=3)))]
  b.global_localizer.call_async.return_value=done(None);b.nomotion.service_is_ready.return_value=False
  async def sleep(_):
   if b.amcl_lifecycle.call_async.call_count==1:b.global_localizer.call_async.assert_not_called()
  with patch('ros_bridge.asyncio.sleep',sleep):await RosBridge.auto_localize(b)
  self.assertEqual(b.amcl_lifecycle.call_async.call_count,2);b.global_localizer.call_async.assert_called_once()
 async def test_missing_map_never_calls_global(self):
  b=self.bridge();b.map_meta=None
  async def sleep(_):b.localization_epoch+=1
  with patch('ros_bridge.asyncio.sleep',sleep):await RosBridge.auto_localize(b)
  b.global_localizer.call_async.assert_not_called();b.amcl_lifecycle.call_async.assert_not_called()
 async def test_cancel_during_state_query_never_calls_global(self):
  b=self.bridge()
  def reply(_):b.localization_epoch+=1;return done(NS(current_state=NS(id=3)))
  b.amcl_lifecycle.call_async.side_effect=reply
  await RosBridge.auto_localize(b);b.global_localizer.call_async.assert_not_called()

 async def test_deferred_initialization_retries_until_first_pose(self):
  b=self.bridge();b.amcl_lifecycle.call_async.return_value=done(NS(current_state=NS(id=3)))
  b.nomotion.service_is_ready.return_value=False
  def init(_):
   if b.global_localizer.call_async.call_count==2:b.amcl={'pose':True}
   return done(None)
  b.global_localizer.call_async.side_effect=init;b.localized.side_effect=lambda:b.amcl is not None
  async def sleep(_):pass
  with patch('ros_bridge.asyncio.sleep',sleep):await RosBridge.auto_localize(b)
  self.assertEqual(b.global_localizer.call_async.call_count,2)
