"""Persistent console-specific navigation policy; never edits vendor defaults."""
import re
from pathlib import Path

from core import circle_footprint, footprint_center, footprint_text


def available_mppi_critics():
    """当前安装的 MPPI 提供了哪些 critic（读插件描述文件）；读不到就返回空集合。

    这台车的 nav2_mppi_controller 是 Humble 版本，只有 9 个 critic，
    没有 VelocityDeadbandCritic / CostCritic。写进配置不存在的插件会让
    controller_server configure 失败，连累整个 Navi2 bringup 起不来。"""
    global _MPPI_CRITICS
    if _MPPI_CRITICS is not None: return _MPPI_CRITICS
    import glob, re as _re
    found = set()
    for pattern in ('/home/wheeltec/wheeltec_ros2/install/nav2_mppi_controller/share/nav2_mppi_controller/critics.xml',
                    '/opt/ros/*/share/nav2_mppi_controller/critics.xml',
                    '/home/wheeltec/wheeltec_ros2/src/**/critics.xml'):
        for path in glob.glob(pattern, recursive=True):
            try:
                found |= set(_re.findall(r'mppi::critics::([A-Za-z]+Critic)', open(path).read()))
            except OSError:
                continue
    _MPPI_CRITICS = found
    return found


_MPPI_CRITICS = None


def apply_forward_profile(cfg,root,arrival_radius=.20,clearance=.22,min_turn_radius=.30,avoidance=True):
    """前向导航配置 + 连续巡航行为树。

    arrival_radius 是“算作到达航点”的半径（行为树的经过半径用它；终点停靠容差另取，见下），
    规划器容差取它的 2/3（且不超过 0.25），否则规划出的路径可能停在航点 0.25 m 外，
    反而永远满足不了经过判定，小车就会绕回去。

    clearance 是障碍物周边的**硬禁区**半径：把有效足迹换成半径 clearance 的圆，
    代价地图把该范围内的栅格标成内切/致命，规划器无法进入。它必须小于真实可通行余量，
    否则车（或其估计位置）一旦靠近墙，规划器就会报 "Starting point in lethal space" 直接失败。
    软避让交给膨胀半径（≥0.45 m），让规划器在“很贵但可通行”的代价下优先挑宽敞通道。"""
    planner=cfg['planner_server']['ros__parameters']['GridBased']
    controller=cfg['controller_server']['ros__parameters']['FollowPath']
    arrival_radius=max(.05,min(float(arrival_radius),1.))
    planner['motion_model_for_search']='DUBIN'
    planner['tolerance']=round(max(.05,min(arrival_radius*2/3,.25)),3)
    # 实测最小转弯半径 0.30 m：规划器与控制器都按它算，才敢走窄一点的弯
    min_turn=max(.15,min(float(min_turn_radius),1.))
    # 注意层级：SmacPlannerHybrid 的参数就写在 GridBased 这一层（planner 自身），
    # 再 setdefault('GridBased') 会新建一层嵌套，值根本落不到规划器上。
    if 'minimum_turning_radius' in planner or planner.get('motion_model_for_search'):
        planner['minimum_turning_radius']=round(min_turn,3)
    follow_cfg=cfg['controller_server']['ros__parameters'].setdefault('FollowPath',{})
    if follow_cfg.get('AckermannConstraints') is not None or follow_cfg.get('motion_model')=='Ackermann':
        follow_cfg.setdefault('AckermannConstraints',{})['min_turning_r']=round(min_turn,3)
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
    clearance=max(.05,min(float(clearance),1.5))
    if not avoidance:
        # 贴线巡航（用户要求“把致命栅格关了”）：硬禁区收到车体半宽（0.10 m），
        # 于是"内切/致命"只存在于障碍本身附近，规划器不会因为贴近墙而拒绝起步；
        # 软避让只留 0.20 m 薄薄一层代价，路径会贴着教学线走。碰撞风险由用户承担。
        clearance=.10
        inflation_radius=.20
    else:
        inflation_radius=max(clearance*1.5,.45)
    for node in ('global_costmap','local_costmap'):
        params=_costmap_params(cfg,node)
        if params is None:continue
        center=footprint_center(params.get('footprint'))
        params.pop('robot_radius',None)
        params['footprint']=footprint_text(circle_footprint(clearance,center))
        layer=params.setdefault('inflation_layer',{})
        layer['enabled']=True
        # 硬禁区（内切半径=足迹半径）要小，否则车稍微靠近墙就“起点在致命区”导致规划失败；
        # 软避让（膨胀）留大，规划器仍会主动挑 ≥45 cm 的通道。
        layer['inflation_radius']=round(inflation_radius,2)
        layer.setdefault('cost_scaling_factor',3.0)
    # AMCL：默认参数在这个场地上偏保守——走 25 cm 才更新一次、自恢复被关死（收敛到错位置就回不来）
    amcl=cfg.setdefault('amcl',{}).setdefault('ros__parameters',{})
    amcl.setdefault('scan_topic','scan')
    amcl['update_min_d']=0.10        # 由 0.25 收紧：走 10 cm 就更新一次滤波器
    amcl['update_min_a']=0.10        # 由 0.20 收紧
    amcl['max_particles']=3000       # 由 2000 提高，转弯/遮挡时更稳
    amcl['recovery_alpha_slow']=0.001   # 打开自恢复：粒子跑偏后能重新撒开
    amcl['recovery_alpha_fast']=0.1
    amcl['max_beams']=120            # 由 60 提高：似然场用更多光束
    amcl['laser_max_range']=11.5     # 比雷达量程略小，超过的读数按无效丢弃（默认 100 会把"无回波"当有效）
    amcl['laser_model_type']='likelihood_field'
    amcl['laser_likelihood_max_dist']=2.0
    # 两件事必须分开：
    #  - 行为树“经过半径”要小（密集航点才分得开，默认 8 cm）
    #  - 终点“停靠容差”要留余量，否则车到不了那么准，就会在点旁一直磨轮子
    passed_radius=arrival_radius
    stop_radius=round(min(.25,max(passed_radius,.20)),3)
    goal_checker=cfg['controller_server']['ros__parameters'].setdefault('goal_checker',{})
    goal_checker.update({'xy_goal_tolerance':stop_radius,'yaw_goal_tolerance':3.141592653589793,'stateful':True})
    controller.setdefault('GoalAngleCritic',{})['enabled']=False
    # MPPI 加速度死区：抑制“一点点前进 + 左右修方向”的持续微调。
    # 只有当前构建确实提供该 critic 时才加——加不存在的插件会让控制器 configure 直接 FATAL，
    # 进而 lifecycle_manager 中止整个 Nav2 bringup（预览直接超时）。
    follow=cfg['controller_server']['ros__parameters'].setdefault('FollowPath',{})
    critics=list(follow.get('critics') or [])
    if critics and 'VelocityDeadbandCritic' in available_mppi_critics():
        if 'VelocityDeadbandCritic' not in critics:
            critics.insert(critics.index('GoalCritic') if 'GoalCritic' in critics else len(critics),'VelocityDeadbandCritic')
            follow['critics']=critics
        follow.setdefault('VelocityDeadbandCritic',{}).update({'enabled':True,'cost_power':1,'cost_weight':1.0,
                                                               'deadband_velocity':[.05,0.,.2]})
    return cfg


def _costmap_params(cfg,node):
    """Nav2 代价地图参数在 <node>:<node>:ros__parameters 下，兼容两种写法。"""
    block=cfg.get(node)
    if not isinstance(block,dict):return None
    if 'ros__parameters' in block:return block['ros__parameters']
    for value in block.values():
        if isinstance(value,dict) and 'ros__parameters' in value:return value['ros__parameters']
    return None
