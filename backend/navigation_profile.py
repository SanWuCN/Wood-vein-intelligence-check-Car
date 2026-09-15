"""Persistent console-specific navigation policy; never edits vendor defaults."""
import re
from pathlib import Path


def apply_forward_profile(cfg,root,arrival_radius=.25):
    """前向导航配置 + 连续巡航行为树。

    arrival_radius 是“算作到达航点”的半径：行为树的经过半径与终点 goal_checker 都用它，
    规划器容差取它的 2/3（且不超过 0.25），否则规划出的路径可能停在航点 0.25 m 外，
    反而永远满足不了经过判定，小车就会绕回去。"""
    planner=cfg['planner_server']['ros__parameters']['GridBased']
    controller=cfg['controller_server']['ros__parameters']['FollowPath']
    arrival_radius=max(.05,min(float(arrival_radius),1.))
    planner['motion_model_for_search']='DUBIN'
    planner['tolerance']=round(max(.05,min(arrival_radius*2/3,.25)),3)
    controller['vx_min']=0.0
    controller['enforce_path_inversion']=False
    # This installed Humble critic uses forward_preference, not the newer mode enum.
    controller.setdefault('PathAngleCritic',{})['forward_preference']=True
    controller['PathAngleCritic']['mode']=0
    cfg['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(Path(root)/'backend/behavior_trees/forward_navigation.xml')
    # 连续巡航行为树的经过半径写入运行副本，避免改一个数字还要碰模板
    template=(Path(root)/'backend/behavior_trees/continuous_navigation.xml').read_text()
    radius_text=('%g'%arrival_radius)
    rendered=re.sub(r'(RemovePassedGoals[^>]*radius=")[^"]*(")',
                    lambda m:m.group(1)+radius_text+m.group(2),template,count=1)
    runtime=Path(root)/'runtime'/'behavior_trees';runtime.mkdir(parents=True,exist_ok=True)
    target=runtime/'continuous_navigation.xml';target.write_text(rendered)
    cfg['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml']=str(target)
    goal_checker=cfg['controller_server']['ros__parameters'].setdefault('goal_checker',{})
    goal_checker.update({'xy_goal_tolerance':arrival_radius,'yaw_goal_tolerance':3.141592653589793,'stateful':True})
    controller.setdefault('GoalAngleCritic',{})['enabled']=False
    return cfg
