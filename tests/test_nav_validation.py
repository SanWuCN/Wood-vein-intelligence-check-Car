"""Exercise real bridge entry points without constructing a ROS node or publishing."""
import asyncio
from concurrent.futures import Future
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
from core import ConsoleError
try:
 from ros_bridge import RosBridge
except ImportError:
 RosBridge=None
try:
 from geometry_msgs.msg import PoseStamped
except ImportError:
 PoseStamped=None


def done(value):
    f=Future();f.set_result(value);return f

def pose_msg(p):
    msg=PoseStamped();msg.header.frame_id='map'
    msg.pose.position.x=float(p['x']);msg.pose.position.y=float(p['y']);msg.pose.orientation.w=1.0
    return msg

@unittest.skipIf(RosBridge is None,'Run on robot with system ROS libraries')
class NavigationValidationTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.points=[{'x':1.0,'y':1.0},{'x':2.0,'y':1.0}]
        self.requests=[]
        def send(req):
            self.requests.append(req)
            result=SimpleNamespace(status=4,result=SimpleNamespace(path=SimpleNamespace(poses=req.goals)))
            return done(SimpleNamespace(accepted=True,get_result_async=lambda:done(result)))
        self.b=SimpleNamespace(mode='navigation',localized=lambda:True,lock=threading.RLock(),
            map_meta={'width':5,'height':5,'resolution':1.0,'origin':{'x':0,'y':0,'yaw':0}},
            grid=np.zeros((5,5),dtype=np.int16),plan=[],pose={'x':0.,'y':0.,'yaw':0.},pose_msg=pose_msg,
            arrival_radius=.15,lookahead=4,clearance=.30,speed=.05,
            planner=SimpleNamespace(server_is_ready=lambda:True,send_goal_async=send),
            nav=SimpleNamespace(server_is_ready=lambda:True),mission={'state':'stopped'},epoch=0,error=None,
            publish_speed=Mock(),_send_nav=Mock(),active_clearance=lambda:.1,
            nav_stack_up=lambda:True,mission_trace=Mock())
    async def test_loop_preview_validates_and_closes_route(self):
        result=await RosBridge.preview(self.b,self.points,'loop')
        self.assertEqual(result['point_count'],2);self.assertEqual(len(result['path']),3)
        self.assertEqual([r.use_start for r in self.requests],[False])
        self.assertEqual(result['path'][0],result['path'][-1]);self.b._send_nav.assert_not_called()
    async def test_preview_rejects_obstacle_before_planning(self):
        self.b.grid[1,1]=100
        with self.assertRaises(ConsoleError) as error:await RosBridge.preview(self.b,self.points,'multi')
        self.assertEqual(error.exception.code,'POINT_BLOCKED');self.assertEqual(self.requests,[])
    async def test_start_uses_validated_points_with_mock_sender(self):
        result=await RosBridge.start_mission(self.b,self.points,'multi')
        self.assertEqual(result['state'],'accepting');self.assertEqual(result['points'],self.points)
        self.b._send_nav.assert_called_once_with(1);self.b.publish_speed.assert_called_once()
    async def test_start_rejects_unknown_without_dispatch(self):
        self.b.grid[1,1]=-1
        with self.assertRaises(ConsoleError) as error:await RosBridge.start_mission(self.b,self.points,'multi')
        self.assertEqual(error.exception.code,'POINT_BLOCKED');self.b._send_nav.assert_not_called();self.b.publish_speed.assert_not_called()
        self.assertEqual(self.b.mission['state'],'stopped')

if __name__=='__main__':unittest.main()
