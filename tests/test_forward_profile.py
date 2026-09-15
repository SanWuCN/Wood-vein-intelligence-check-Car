import sys, unittest, copy
from pathlib import Path
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
from navigation_profile import apply_forward_profile
class ForwardProfileTests(unittest.TestCase):
 def test_planner_controller_and_recovery_agree(self):
  root=Path(__file__).resolve().parents[1]
  cfg={'planner_server':{'ros__parameters':{'GridBased':{'motion_model_for_search':'REEDS_SHEPP','minimum_turning_radius':.35}}},'controller_server':{'ros__parameters':{'FollowPath':{'vx_min':-.35,'vx_max':.35,'motion_model':'Ackermann'}}},'bt_navigator':{'ros__parameters':{}}}
  apply_forward_profile(cfg,root)
  self.assertEqual(cfg['planner_server']['ros__parameters']['GridBased']['motion_model_for_search'],'DUBIN')
  c=cfg['controller_server']['ros__parameters']['FollowPath'];self.assertEqual(c['vx_min'],0);self.assertEqual(c['vx_max'],.35)
  self.assertTrue(c['PathAngleCritic']['forward_preference']);self.assertFalse(c['enforce_path_inversion'])
  bt=ET.parse(cfg['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']);tags={n.tag for n in bt.iter()}
  self.assertTrue({'ComputePathToPose','FollowPath','Wait','ClearEntireCostmap'}<=tags)
  self.assertFalse(tags&{'BackUp','Spin','DriveOnHeading'})
  checker=cfg['controller_server']['ros__parameters']['goal_checker']
  self.assertEqual(checker['xy_goal_tolerance'],.4);self.assertGreaterEqual(checker['yaw_goal_tolerance'],3.14159)
  self.assertFalse(c['GoalAngleCritic']['enabled'])
  through=ET.parse(cfg['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'])
  self.assertIsNotNone(through.find('.//ComputePathThroughPoses'))
  self.assertEqual(through.find('.//RemovePassedGoals').get('radius'),'0.40')
  self.assertFalse({n.tag for n in through.iter()}&{'BackUp','Spin'})
  prior=copy.deepcopy(cfg);apply_forward_profile(cfg,root);self.assertEqual(cfg,prior)
