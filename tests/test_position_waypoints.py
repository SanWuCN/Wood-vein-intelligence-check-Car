import math,sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
from core import validate_waypoints,automatic_goals,ConsoleError
class PositionWaypointTests(unittest.TestCase):
 def test_positions_only_and_legacy_heading_ignored(self):
  meta={'width':5,'height':5,'resolution':1,'origin':{'x':0,'y':0,'yaw':0}}
  self.assertEqual(validate_waypoints([{'x':1,'y':2,'yaw':float('nan')}],meta,np.zeros((5,5)),'single'),[{'x':1.,'y':2.}])
 def test_corner_tangent_and_final_incoming(self):
  points=[{'x':1,'y':0},{'x':1,'y':1}];result=automatic_goals(points,{'x':0,'y':0,'yaw':2})
  self.assertAlmostEqual(result[0]['yaw'],math.pi/4);self.assertAlmostEqual(result[1]['yaw'],math.pi/2)
  self.assertEqual(points,[{'x':1,'y':0},{'x':1,'y':1}])
 def test_loop_uses_closing_segment_not_robot_heading(self):
  ps=[{'x':0,'y':0},{'x':1,'y':0},{'x':1,'y':1},{'x':0,'y':1}]
  a=automatic_goals(ps,{'x':100,'y':100,'yaw':2},'loop');b=automatic_goals(ps,None,'loop')
  self.assertEqual(a,b);self.assertAlmostEqual(a[0]['yaw'],-math.pi/4)
 def test_single_goal_and_duplicate_are_finite(self):
  self.assertAlmostEqual(automatic_goals([{'x':0,'y':1}],{'x':0,'y':0,'yaw':-1})[0]['yaw'],math.pi/2)
  self.assertTrue(all(math.isfinite(p['yaw']) for p in automatic_goals([{'x':1,'y':1}]*2,{'x':1,'y':1,'yaw':.2},'loop')))
