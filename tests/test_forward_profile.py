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
  self.assertFalse(tags&{'Spin','DriveOnHeading'})   # 原地旋转会造成绕圈，倒车恢复是允许的
  checker=cfg['controller_server']['ros__parameters']['goal_checker']
  # 默认到点半径 0.20（实测最小转弯半径 0.30 m）：停靠容差不小于它
  self.assertEqual(checker['xy_goal_tolerance'],.20);self.assertGreaterEqual(checker['yaw_goal_tolerance'],3.14159)
  self.assertEqual(cfg['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'],.30)
  self.assertEqual(c['AckermannConstraints']['min_turning_r'],.30)
  self.assertFalse(c['GoalAngleCritic']['enabled'])
  # 规划器容差必须小于到点半径，否则路径不会真的经过航点
  self.assertLessEqual(cfg['planner_server']['ros__parameters']['GridBased']['tolerance'],.20)
  through=ET.parse(cfg['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'])
  self.assertIsNotNone(through.find('.//ComputePathThroughPoses'))
  self.assertEqual(through.find('.//RemovePassedGoals').get('radius'),'0.2')
  self.assertIn('runtime',cfg['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'])
  self.assertFalse({n.tag for n in through.iter()}&{'Spin'})
  prior=copy.deepcopy(cfg);apply_forward_profile(cfg,root);self.assertEqual(cfg,prior)
 def test_obstacle_clearance_becomes_effective_footprint(self):
  root=Path(__file__).resolve().parents[1]
  body='[ [-0.031, -0.093], [-0.031, 0.093], [0.209, 0.093], [0.209, -0.093] ]'   # 厂商参数就是字符串写法
  cfg={'planner_server':{'ros__parameters':{'GridBased':{}}},'controller_server':{'ros__parameters':{'FollowPath':{}}},'bt_navigator':{'ros__parameters':{}},
       'global_costmap':{'global_costmap':{'ros__parameters':{'footprint':body,'inflation_layer':{'inflation_radius':.1}}}},
       'local_costmap':{'local_costmap':{'ros__parameters':{'footprint':body,'inflation_layer':{'inflation_radius':.1}}}}}
  apply_forward_profile(cfg,root,.25,.30)
  for node in ('global_costmap','local_costmap'):
   from core import footprint_points
   p=cfg[node][node]['ros__parameters'];self.assertIsInstance(p['footprint'],str)   # 写回字符串，Nav2 按原方式解析
   pts=footprint_points(p['footprint']);xs=[q[0] for q in pts];ys=[q[1] for q in pts]
   self.assertAlmostEqual((max(xs)-min(xs))/2,.30,places=3)      # 内切半径=禁区半径 → 30cm 内不可规划
   self.assertAlmostEqual((max(xs)+min(xs))/2,.089,places=3)     # 圆套在车体几何中心
   self.assertAlmostEqual(p['inflation_layer']['inflation_radius'],.45,places=3)
 def test_clearance_is_configurable(self):
  root=Path(__file__).resolve().parents[1]
  cfg={'planner_server':{'ros__parameters':{'GridBased':{}}},'controller_server':{'ros__parameters':{'FollowPath':{}}},'bt_navigator':{'ros__parameters':{}},
       'local_costmap':{'local_costmap':{'ros__parameters':{'footprint':[[0,0],[1,0],[1,1],[0,1]]}}}}
  apply_forward_profile(cfg,root,.25,.5)
  from core import footprint_points
  p=cfg['local_costmap']['local_costmap']['ros__parameters']
  xs=[q[0] for q in footprint_points(p['footprint'])]
  self.assertAlmostEqual((max(xs)-min(xs))/2,.5,places=3)
 def test_reverse_recovery_present(self):
  """离墙太近/规划失败时要有有界倒车恢复（BackUp），但不能有原地旋转。"""
  root=Path(__file__).resolve().parents[1]
  cfg={'planner_server':{'ros__parameters':{'GridBased':{}}},'controller_server':{'ros__parameters':{'FollowPath':{}}},'bt_navigator':{'ros__parameters':{}}}
  apply_forward_profile(cfg,root)
  bt=ET.parse(cfg['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'])
  back=bt.find('.//BackUp')
  self.assertIsNotNone(back)
  self.assertLessEqual(float(back.get('backup_dist')),.30)     # 有界，避免倒太远
  self.assertLessEqual(float(back.get('backup_speed')),.10)
  self.assertIsNone(bt.find('.//Spin'))
 def test_dense_route_keeps_small_passed_radius_but_stops_comfortably(self):
  root=Path(__file__).resolve().parents[1]
  cfg={'planner_server':{'ros__parameters':{'GridBased':{}}},'controller_server':{'ros__parameters':{'FollowPath':{'critics':['GoalCritic','PathFollowCritic']}}},'bt_navigator':{'ros__parameters':{}}}
  apply_forward_profile(cfg,root,.08)
  # 经过半径保持 8 cm（密集航点才分得开），停靠容差放宽到 20 cm（不在点旁磨轮子）
  self.assertEqual(cfg['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance'],.20)
  bt=ET.parse(root/'runtime/behavior_trees/continuous_navigation.xml').getroot()
  self.assertEqual(bt.find('.//RemovePassedGoals').get('radius'),'0.08')
 def test_never_adds_critic_missing_from_build(self):
  """构建里没有的 critic 绝不能写进配置：会让 controller_server FATAL 并中止整个 bringup。"""
  import navigation_profile as np
  root=Path(__file__).resolve().parents[1]
  cfg={'planner_server':{'ros__parameters':{'GridBased':{}}},'controller_server':{'ros__parameters':{'FollowPath':{'critics':['GoalCritic','PathFollowCritic']}}},'bt_navigator':{'ros__parameters':{}}}
  saved=np._MPPI_CRITICS
  try:
   np._MPPI_CRITICS=set()
   apply_forward_profile(cfg,root,.08)
   follow=cfg['controller_server']['ros__parameters']['FollowPath']
   self.assertEqual(follow['critics'],['GoalCritic','PathFollowCritic'])
   self.assertNotIn('VelocityDeadbandCritic',follow)
   np._MPPI_CRITICS={'GoalCritic','VelocityDeadbandCritic'};cfg['controller_server']['ros__parameters']['FollowPath']['critics']=['GoalCritic','PathFollowCritic']
   apply_forward_profile(cfg,root,.08)
   self.assertIn('VelocityDeadbandCritic',follow['critics'])
   self.assertEqual(follow['critics'].count('VelocityDeadbandCritic'),1)
   apply_forward_profile(cfg,root,.08)
   self.assertEqual(follow['critics'].count('VelocityDeadbandCritic'),1)   # 幂等
  finally:
   np._MPPI_CRITICS=saved
 def test_arrival_radius_is_configurable(self):
  root=Path(__file__).resolve().parents[1]
  cfg={'planner_server':{'ros__parameters':{'GridBased':{}}},'controller_server':{'ros__parameters':{'FollowPath':{}}},'bt_navigator':{'ros__parameters':{}}}
  apply_forward_profile(cfg,root,.35)
  # 停靠容差有 25 cm 上限（到点判定仍按传入值）
  self.assertEqual(cfg['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance'],.25)
  self.assertEqual(cfg['planner_server']['ros__parameters']['GridBased']['tolerance'],.233)
  import xml.etree.ElementTree as ET
  bt=ET.parse(cfg['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'])
  self.assertEqual(bt.find('.//RemovePassedGoals').get('radius'),'0.35')
