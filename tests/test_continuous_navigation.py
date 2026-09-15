"""Real bridge callbacks with memory-only actions: never constructs a ROS node.

连续巡航按“目标窗口”分批下发：窗口内航点依次穿过（RemovePassedGoals 0.40 m），
整批走完才推进游标。循环模式窗口必须小于一圈——控制器每个周期都拿路径终点与
车位比较，终点若正好落在车位会被立刻判为到达，循环就会空转。

注意：每次下发都要用新的 result future，否则已完成的 future 会让
_finished → _send_nav → 立刻完成 → _finished 递归下去。
分批规则本身的纯逻辑测试见 test_mission_batching.py（本机可跑）。
"""
import sys,threading,unittest
from pathlib import Path
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
try:
 from ros_bridge import RosBridge
 from geometry_msgs.msg import PoseStamped
except ImportError:
 RosBridge=None

def done(value):f=Future();f.set_result(value);return f

def pose(p):
 m=PoseStamped();m.pose.position.x=float(p['x']);m.pose.position.y=float(p['y']);m.pose.orientation.w=1.;return m

def goals_of(points):return [{'x':float(x),'y':float(y),'yaw':0.} for x,y in points]

@unittest.skipIf(RosBridge is None,'Run on robot with system ROS libraries')
class ContinuousTests(unittest.TestCase):
 def setup_bridge(self,mode='multi',lookahead=0,points=None):
  self.sent=[];self.feedback=None;self.handles=[]
  def send(req,feedback_callback):
   self.sent.append(req);self.feedback=feedback_callback
   result=Future()                       # 每次下发独立 future，模拟真实的一次动作往返
   handle=NS(accepted=True,get_result_async=lambda:result,cancel_goal_async=Mock())
   self.handles.append(result)
   return done(handle)
  pts=points or [(1,0),(2,1),(3,1)]
  b=NS(epoch=2,lock=threading.RLock(),route_cursor=0,pose_msg=pose,goal_handle=None,pending_goal=None,lookahead=lookahead,
       lookahead_m=0.,lookahead_max=0,skip_missed=False,_skipping=False,_last_skip_at=0.,
       mission={'state':'accepting','index':0,'cycle':0,'mode':mode,'points':[{'x':x,'y':y} for x,y in pts]},
       navigation_goals=goals_of(pts),error=None,nav=NS(send_goal_async=send))
  b._finished=lambda f,t:RosBridge._finished(b,f,t)
  b._send_nav=lambda t:RosBridge._send_nav(b,t)
  b._batch_goals=lambda s:RosBridge._batch_goals(b,s)
  b._waypoint_index=lambda:RosBridge._waypoint_index(b)
  return b
 def finish(self,status=4):self.handles[-1].set_result(NS(status=status))
 def test_sends_future_waypoints_and_progress_never_rewinds(self):
  b=self.setup_bridge();b._send_nav(2)
  self.assertEqual(len(self.sent[0].poses),3)
  self.feedback(NS(feedback=NS(number_of_poses_remaining=2,distance_remaining=2.)))
  self.assertEqual(b.route_cursor,1);self.assertEqual(b.mission['index'],1)
  self.feedback(NS(feedback=NS(number_of_poses_remaining=3,distance_remaining=3.)))
  self.assertEqual(b.route_cursor,1)
  b._send_nav(2);self.assertEqual(len(self.sent[1].poses),2);self.assertEqual(self.sent[1].poses[0].pose.position.x,2.)
 def test_window_bounds_planning_and_advances_batch_by_batch(self):
  b=self.setup_bridge('multi',lookahead=2,points=[(1,0),(2,0),(3,0),(4,0),(5,0)])
  b._send_nav(2)
  self.assertEqual([p.pose.position.x for p in self.sent[0].poses],[1.,2.])
  self.finish()
  self.assertEqual(b.route_cursor,2);self.assertEqual(b.mission['state'],'running')
  self.assertEqual([p.pose.position.x for p in self.sent[1].poses],[3.,4.])
  self.finish()
  self.assertEqual(b.route_cursor,4)
  self.assertEqual([p.pose.position.x for p in self.sent[2].poses],[5.])
  self.finish()
  self.assertEqual(b.mission['state'],'completed');self.assertEqual(b.mission['distance_remaining'],0.)
  self.assertEqual(len(self.sent),3)
 def test_loop_window_is_shorter_than_one_lap(self):
  # 一圈 3 点时窗口最多 2 个：终点不能正好是车位
  b=self.setup_bridge('loop',points=[(1,0),(2,1),(3,1)])
  b._send_nav(2)
  self.assertEqual([p.pose.position.x for p in self.sent[0].poses],[1.,2.])
 def test_loop_rolls_over_without_revisiting_or_resetting(self):
  b=self.setup_bridge('loop',points=[(1,0),(2,1),(3,1)])
  b._send_nav(2);self.finish()
  self.assertEqual(b.route_cursor,2);self.assertEqual(b.mission['cycle'],0)
  self.assertEqual([p.pose.position.x for p in self.sent[1].poses],[3.,1.])   # 环状取点覆盖闭合段
  self.finish()
  self.assertEqual(b.route_cursor,4);self.assertEqual(b.mission['cycle'],1)  # 4//3
  self.assertEqual([p.pose.position.x for p in self.sent[2].poses],[2.,3.])
  self.assertEqual(len(self.sent),3)
 def test_stale_feedback_and_result_do_not_restart(self):
  b=self.setup_bridge();b._send_nav(2);b.epoch=3;b.mission['state']='paused'
  self.feedback(NS(feedback=NS(number_of_poses_remaining=1,distance_remaining=0.)))
  self.finish()
  self.assertEqual(b.route_cursor,0);self.assertEqual(b.mission['state'],'paused');self.assertEqual(len(self.sent),1)
 def test_success_completes_whole_route(self):
  b=self.setup_bridge();b._send_nav(2);self.finish()
  self.assertEqual(b.mission['state'],'completed');self.assertEqual(len(self.sent),1)
 def test_failure_never_advances_or_restarts(self):
  b=self.setup_bridge();b._send_nav(2);self.finish(6)
  self.assertEqual(b.mission['state'],'failed');self.assertEqual(len(self.sent),1)
