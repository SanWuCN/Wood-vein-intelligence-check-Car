"""Persistent console-specific navigation policy; never edits vendor defaults."""
from pathlib import Path

def apply_forward_profile(cfg,root):
    planner=cfg['planner_server']['ros__parameters']['GridBased']
    controller=cfg['controller_server']['ros__parameters']['FollowPath']
    planner['motion_model_for_search']='DUBIN'
    controller['vx_min']=0.0
    controller['enforce_path_inversion']=False
    # This installed Humble critic uses forward_preference, not the newer mode enum.
    controller.setdefault('PathAngleCritic',{})['forward_preference']=True
    controller['PathAngleCritic']['mode']=0
    cfg['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(Path(root)/'backend/behavior_trees/forward_navigation.xml')
    cfg['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml']=str(Path(root)/'backend/behavior_trees/continuous_navigation.xml')
    goal_checker=cfg['controller_server']['ros__parameters'].setdefault('goal_checker',{})
    goal_checker.update({'xy_goal_tolerance':0.40,'yaw_goal_tolerance':3.141592653589793,'stateful':True})
    controller.setdefault('GoalAngleCritic',{})['enabled']=False
    return cfg
