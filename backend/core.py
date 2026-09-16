"""ROS-independent geometry, persistence and input validation."""
import io
import json
import math
import re
import time
import uuid
from pathlib import Path
import numpy as np
import yaml
from PIL import Image


class ConsoleError(Exception):
    def __init__(self, code, message, status=409):
        super().__init__(message)
        self.code, self.status = code, status


def finite(value, name, low=-10000, high=10000):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ConsoleError('INVALID_ARGUMENT', f'{name} 必须为有限数值', 422)
    if not low <= value <= high:
        raise ConsoleError('OUT_OF_RANGE', f'{name} 超出范围', 422)
    return float(value)


def pose_arg(value):
    if not isinstance(value, dict):
        raise ConsoleError('INVALID_POSE', '坐标格式错误', 422)
    return {k: finite(value.get(k), k, -math.pi if k == 'yaw' else -10000,
                      math.pi if k == 'yaw' else 10000) for k in ('x', 'y', 'yaw')}


def world_to_cell(meta, x, y):
    o = meta['origin']; a = o['yaw']; dx, dy = x-o['x'], y-o['y']
    return math.floor((math.cos(a)*dx+math.sin(a)*dy)/meta['resolution']), math.floor((-math.sin(a)*dx+math.cos(a)*dy)/meta['resolution'])


def cell_to_world(meta, x, y):
    o = meta['origin']; a = o['yaw']; gx, gy = x*meta['resolution'], y*meta['resolution']
    return o['x']+math.cos(a)*gx-math.sin(a)*gy, o['y']+math.sin(a)*gx+math.cos(a)*gy


MIN_WAYPOINT_SPACING = 0.02


def validate_waypoints(points, meta, grid, mode='multi'):
    if mode not in ('single', 'multi', 'loop'):
        raise ConsoleError('INVALID_MODE', '巡航模式无效', 422)
    if not isinstance(points, list) or not 1 <= len(points) <= 100:
        raise ConsoleError('INVALID_WAYPOINTS', '航点数量应为 1–100', 422)
    if (mode == 'single' and len(points) != 1) or (mode == 'loop' and len(points) < 2):
        raise ConsoleError('INVALID_WAYPOINTS', '航点数量与巡航模式不符', 422)
    if meta is None or grid is None:
        raise ConsoleError('MAP_UNAVAILABLE', '地图未就绪')
    result = []
    for p in points:
        if not isinstance(p, dict):raise ConsoleError('INVALID_WAYPOINTS', '航点必须为坐标对象', 422)
        result.append({'x':finite(p.get('x'),'x',-100000,100000), 'y':finite(p.get('y'),'y',-100000,100000)})
    for p in result:
        x, y = world_to_cell(meta, p['x'], p['y'])
        if not (0 <= x < meta['width'] and 0 <= y < meta['height']):
            raise ConsoleError('OUTSIDE_MAP', '航点超出地图', 422)
        v = int(grid[y, x])
        if v < 0 or v > 20:
            raise ConsoleError('POINT_BLOCKED', '航点位于障碍物或未知区域', 422)
    # 相邻航点（循环含闭合段）必须拉开距离：太近时连续导航窗口的终点会落在车位上，
    # 控制器每个周期都拿路径终点与车位比较，会立刻判为到达并跳过后续航点。
    pairs = list(zip(range(len(result)), result, result[1:] + ([result[0]] if mode == 'loop' and len(result) > 1 else [])))
    for i, a, b in pairs:
        if math.hypot(b['x'] - a['x'], b['y'] - a['y']) < MIN_WAYPOINT_SPACING:
            raise ConsoleError('POINTS_TOO_CLOSE', f'相邻航点 {i+1} 与 {(i+1) % len(result)+1} 距离需 ≥{MIN_WAYPOINT_SPACING:g} m', 422)
    return result


def automatic_goals(points, start, mode='multi'):
    """Resolve position-only waypoints to internal Dubins goals.

    Corner headings bisect incoming/outgoing travel directions; endpoints follow
    their adjacent segment. Nav2 then finds collision-free, curvature-bounded arcs.
    Legacy yaw values are deliberately ignored. This is not a minimum-path proof.
    """
    result=[]
    for i,p in enumerate(points):
        prev=points[i-1] if i else (points[-1] if mode=='loop' else start)
        nxt=points[i+1] if i+1<len(points) else (points[0] if mode=='loop' else None)
        directions=[]
        for a,b in ((prev,p),(p,nxt)):
            if a is None or b is None:continue
            dx=b['x']-a['x'];dy=b['y']-a['y'];length=math.hypot(dx,dy)
            if length>1e-6:directions.append((dx/length,dy/length))
        if not directions:
            angle=(start or {}).get('yaw',0.)
        else:
            x=sum(d[0] for d in directions);y=sum(d[1] for d in directions)
            if math.hypot(x,y)<1e-6:x,y=directions[-1]
            angle=math.atan2(y,x)
        result.append({'x':p['x'],'y':p['y'],'yaw':angle})
    return result


def map_image(grid):
    rgb = np.empty((*grid.shape, 3), dtype=np.uint8)
    rgb[:] = (100, 135, 131)
    rgb[(grid >= 0) & (grid <= 20)] = (239, 242, 240)
    rgb[grid > 20] = (24, 32, 35)
    buf = io.BytesIO(); Image.fromarray(np.flipud(rgb)).save(buf, format='PNG')
    return buf.getvalue()


def bounds(grid):
    ys, xs = np.where(grid >= 0)
    if not len(xs): return [0, 0, grid.shape[1], grid.shape[0]]
    # Bounds use image coordinates (top-left), while OccupancyGrid uses bottom-left.
    return [int(xs.min()), int(grid.shape[0]-1-ys.max()), int(xs.max()+1), int(grid.shape[0]-ys.min())]


class MapStore:
    def __init__(self, path):
        self.path = Path(path); self.path.mkdir(parents=True, exist_ok=True)

    def directory(self, map_id):
        if not isinstance(map_id, str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,80}', map_id):
            raise ConsoleError('INVALID_MAP_ID', '地图编号无效', 422)
        p = self.path / map_id
        if not (p/'metadata.json').is_file():
            raise ConsoleError('MAP_NOT_FOUND', '地图不存在', 404)
        return p

    def list(self):
        result=[]
        for p in self.path.glob('*/metadata.json'):
            try: result.append(json.loads(p.read_text()))
            except (OSError, ValueError): continue
        return sorted(result, key=lambda x:x['created_at'], reverse=True)

    def save(self, name, meta, grid, robot_pose=None):
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 40:
            raise ConsoleError('INVALID_NAME', '地图名称应为 1–40 字', 422)
        if meta is None or grid is None or not np.any(grid >= 0):
            raise ConsoleError('MAP_UNAVAILABLE', '没有可保存的地图')
        map_id = 'map_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
        p = self.path/map_id; p.mkdir()
        try:
            pgm = np.full(grid.shape, 205, np.uint8)
            pgm[(grid >= 0) & (grid <= 20)] = 254; pgm[grid > 20] = 0
            Image.fromarray(np.flipud(pgm)).save(p/'map.pgm')
            np.save(p/'grid.npy', grid, allow_pickle=False)
            o=meta['origin']
            (p/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','mode':'trinary','resolution':meta['resolution'], 'origin':[o['x'],o['y'],o['yaw']], 'negate':0,'occupied_thresh':0.65,'free_thresh':0.196}))
            (p/'preview.png').write_bytes(map_image(grid))
            item={'id':map_id,'name':name.strip(),'created_at':time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'map':meta,'saved_pose':robot_pose, 'area_m2':round(float(np.sum((grid >= 0)&(grid<=20)))*meta['resolution']**2,2)}
            (p/'metadata.json').write_text(json.dumps(item,ensure_ascii=False,indent=2))
            return item
        except Exception:
            import shutil
            shutil.rmtree(p)
            raise

    def load(self, map_id):
        p=self.directory(map_id)
        return json.loads((p/'metadata.json').read_text()), np.load(p/'grid.npy',allow_pickle=False)


# ---- 人工定位吸附：与 ROS 无关的纯几何/打分实现，便于离线测试 ----------------
def grid_likelihood(grid, resolution, radius=5, sigma=.12):
    """离障碍物越近分数越高：障碍格 1.0，向外逐格按高斯衰减，用于位姿打分。"""
    occ = grid > 20
    like = np.zeros(grid.shape, dtype=np.float32)
    like[occ] = 1.0
    cur = occ.copy()
    for r in range(1, radius + 1):
        nxt = np.zeros_like(cur)
        nxt[1:, :] |= cur[:-1, :]; nxt[:-1, :] |= cur[1:, :]
        nxt[:, 1:] |= cur[:, :-1]; nxt[:, :-1] |= cur[:, 1:]
        nxt[1:, 1:] |= cur[:-1, :-1]; nxt[1:, :-1] |= cur[:-1, 1:]
        nxt[:-1, 1:] |= cur[1:, :-1]; nxt[:-1, :-1] |= cur[1:, 1:]
        cur = nxt & (like == 0)
        like[cur] = float(math.exp(-((r * resolution) ** 2) / (2 * sigma ** 2)))
    return like


def world_to_cells(meta, xs, ys):
    """world_to_cell 的向量化版本，返回 (col,row) 整数数组。"""
    o = meta['origin']; a = o['yaw']; dx = xs - o['x']; dy = ys - o['y']
    ca, sa = math.cos(a), math.sin(a); res = meta['resolution']
    return np.floor((ca * dx + sa * dy) / res).astype(np.int32), np.floor((-sa * dx + ca * dy) / res).astype(np.int32)


def pose_score(like, meta, points, x, y, theta, min_points=12):
    """把 base 系下的雷达点按候选位姿投到地图上，返回平均似然（0–1）。"""
    ca, sa = math.cos(theta), math.sin(theta)
    col, row = world_to_cells(meta, points[:, 0] * ca - points[:, 1] * sa + x, points[:, 0] * sa + points[:, 1] * ca + y)
    h, w = like.shape
    ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    if int(ok.sum()) < min_points: return 0.0
    return float(like[row[ok], col[ok]].mean())


def nearest_free(meta, grid, x, y, max_cells=60):
    """点选落在障碍或未知区域时，向外螺旋找到最近可通行栅格中心 (距离, x, y)。"""
    width, height = meta['width'], meta['height']
    cx, cy = world_to_cell(meta, x, y)
    if 0 <= cx < width and 0 <= cy < height and 0 <= int(grid[cy, cx]) <= 20: return None
    for r in range(1, max_cells + 1):
        best = None
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r: continue
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < width and 0 <= ny < height): continue
                if not 0 <= int(grid[ny, nx]) <= 20: continue
                wx, wy = cell_to_world(meta, nx + .5, ny + .5)
                d = math.hypot(wx - x, wy - y)
                if best is None or d < best[0]: best = (d, wx, wy)
        if best: return best
    return None


def snap_pose(meta, grid, points, pose, max_shift=.35, max_yaw_deg=12., step=.05, yaw_step=2., like=None):
    """人工定位吸附：先移出障碍/未知区，再在 ±max_shift / ±max_yaw 内按吻合度微调。

    候选位姿整批向量化打分（一次算完所有平移，再逐个偏航角），
    因此在小车 Jetson 上也是几十毫秒级，不会卡住点击操作。
    返回 dict：pose 为吸附后的位姿，shift_m/shift_deg 为微调量，score 为吻合度，
    reason 在缺雷达(no_scan)/缺地图(no_map)/判据不足(weak_match)时给出。
    只有提升显著（≥0.06 且 ≥15%）且（结果吻合 ≥0.35 或提升 ≥0.20）才移动，
    且候选位姿必须落在可通行栅格内。"""
    x, y, theta = float(pose['x']), float(pose['y']), float(pose['yaw'])
    result = {'applied': False, 'free_shift_m': 0.0, 'shift_m': 0.0, 'shift_deg': 0.0, 'score': None, 'samples': 0, 'reason': None}
    if meta is None or grid is None:
        result.update({'pose': {'x': x, 'y': y, 'yaw': theta}, 'reason': 'no_map'}); return result
    free = nearest_free(meta, grid, x, y)
    if free:
        d, x, y = free
        result['free_shift_m'] = round(float(d), 3); result['applied'] = d > 1e-6
    if points is None or len(points) < 12:
        result.update({'pose': {'x': x, 'y': y, 'yaw': theta}, 'reason': 'no_scan'}); return result
    if like is None: like = grid_likelihood(grid, meta['resolution'])
    points = np.asarray(points, dtype=np.float32)
    base = pose_score(like, meta, points, x, y, theta)
    n_shift = int(round(max_shift / step)); n_yaw = int(round(max_yaw_deg / yaw_step))
    offs = np.arange(-n_shift, n_shift + 1, dtype=np.float32) * step
    ox, oy = np.meshgrid(offs, offs, indexing='ij')
    ox, oy = ox.ravel(), oy.ravel()
    h, w = like.shape
    # 只接受位姿本身落在可通行栅格的候选，避免吸附到墙里（会被地图校验拒绝）
    pc, pr = world_to_cells(meta, x + ox, y + oy)
    inside = (pc >= 0) & (pc < w) & (pr >= 0) & (pr < h)
    pcc, prr = np.clip(pc, 0, w - 1), np.clip(pr, 0, h - 1)
    free = inside & (grid[prr, pcc] >= 0) & (grid[prr, pcc] <= 20)
    best = (base, 0., 0., 0.)
    for iy in range(-n_yaw, n_yaw + 1):
        dth = math.radians(yaw_step * iy)
        ca, sa = math.cos(theta + dth), math.sin(theta + dth)
        rx = points[:, 0] * ca - points[:, 1] * sa
        ry = points[:, 0] * sa + points[:, 1] * ca
        col, row = world_to_cells(meta, rx[None, :] + (x + ox)[:, None], ry[None, :] + (y + oy)[:, None])
        ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        cnt = ok.sum(axis=1)
        vals = np.where(ok, like[np.clip(row, 0, h - 1), np.clip(col, 0, w - 1)], 0.).sum(axis=1)
        scores = np.where(free & (cnt >= 12), vals / np.maximum(cnt, 1), 0.)
        k = int(np.argmax(scores))
        if float(scores[k]) > best[0] + 1e-9:
            best = (float(scores[k]), float(ox[k]), float(oy[k]), float(dth))
    _, dx, dy, dth = best
    # 只有明显更优、且结果本身足够吻合时才移动：环境与地图不符时宁可不吸附
    gain = best[0] - base
    if not (gain >= max(.06, base * .15) and (best[0] >= .35 or gain >= .20)):
        if gain > 1e-6: result['reason'] = 'weak_match'
        dx = dy = dth = 0.
    result.update({'pose': {'x': x + dx, 'y': y + dy, 'yaw': theta + dth}, 'shift_m': round(math.hypot(dx, dy), 3),
                   'shift_deg': round(math.degrees(dth), 1), 'score': round(best[0], 3), 'base_score': round(base, 3), 'samples': int(len(points))})
    result['applied'] = result['applied'] or abs(dx) > 1e-9 or abs(dy) > 1e-9 or abs(dth) > 1e-9
    return result


IDLE_MISSION_STATES = ('idle', 'stopped', 'completed', 'failed')


def idle_refine_decision(state, now):
    """静止自动校准的执行判定，返回 (是否执行, 原因)。

    state 字段：enabled / mode / mission / linear / angular / still_since / last_at /
    interval_s / delay_s / pose_age / match / match_min。
    只在导航模式、无任务、底盘连续静止 delay_s 秒、距上次校准 interval_s 秒、
    位姿新鲜且吻合度不低于下限时才执行；原因用于界面提示与排查。"""
    if not state.get('enabled'): return False, 'disabled'
    if state.get('mode') != 'navigation': return False, 'mode'
    if state.get('mission') not in IDLE_MISSION_STATES: return False, 'mission'
    if abs(state.get('linear') or 0) > .02 or abs(state.get('angular') or 0) > .05: return False, 'moving'
    still = state.get('still_since')
    if still is None: return False, 'moving'
    if now - still < state.get('delay_s', 6.): return False, 'settling'
    if now - (state.get('last_at') or 0.) < state.get('interval_s', 8.): return False, 'wait'
    if (state.get('pose_age') if state.get('pose_age') is not None else 99.) > 2.: return False, 'pose_stale'
    match = state.get('match')
    if match is not None and match < state.get('match_min', .35): return False, 'match_low'
    return True, 'ok'


def remaining_route_distance(goals, cursor, count, mode, path_remaining):
    """剩余总里程估算 = 当前窗口剩余路径 + 之后各航点之间的直线距离。

    连续巡航分批下发目标，动作反馈里的 distance_remaining 只覆盖当前窗口，
    这里补上后续航点，界面显示的“剩余距离”才代表整条路线。"""
    n = len(goals)
    if not n: return 0.
    total = float(path_remaining or 0.)
    if mode == 'loop':
        # 含回到本圈起点的闭合段：循环任务的“剩余”是本圈剩余里程
        rest = [goals[(cursor + count + i) % n] for i in range(max(0, n - count) + 1)]
    else:
        rest = goals[cursor + count:]
    prev = goals[(cursor + count - 1) % n]
    for point in rest:
        total += math.hypot(point['x'] - prev['x'], point['y'] - prev['y'])
        prev = point
    return round(total, 2)


def plan_batch(goals, cursor, mode, lookahead, min_length_m=0., max_points=0):
    """连续巡航的单批目标窗口。

    - multi/single：线性推进，lookahead=0 表示把剩余航点一次下发完
    - loop：按环状滚动取点，且窗口必须小于一圈
    窗口小于一圈是硬性要求：控制器每个周期都拿**路径终点**与车位比较，
    若终点正好是车位（循环闭合处）就会被立刻判为到达，任务空转。

    min_length_m/max_points 用于密集航点（遥控录制每 10 cm 一个点）：
    先按点数取，再按累计长度补足到 min_length_m，最后受 max_points 限制——
    否则 10 cm 的点会让“看 4 个点”只覆盖 40 cm，既看不到前面也没法连续行驶。"""
    n = len(goals)
    if not n: return []
    if mode == 'loop':
        limit = max(1, n - 1)
        count = min(lookahead or limit, limit)
    else:
        limit = max(0, n - cursor)
        count = min(lookahead or limit, limit)
    if count and min_length_m > 0:
        def segment(i):
            a = goals[(cursor + i) % n] if mode == 'loop' else goals[cursor + i]
            b = goals[(cursor + i + 1) % n] if mode == 'loop' else goals[cursor + i + 1]
            return math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        travelled = sum(segment(i) for i in range(count - 1))
        index = count
        while index < limit and travelled < min_length_m:   # 航点密集时按距离补足
            travelled += segment(index - 1)
            index += 1
        count = index
    if max_points:
        count = min(count, max(1, max_points), limit)
    if mode == 'loop': return [goals[(cursor + i) % n] for i in range(count)]
    return goals[cursor:cursor + max(0, count)]


def missed_waypoint(robot, target, direction, arrival_radius, min_beyond=.05):
    """航点是否已被“越过”而应当跳过。

    点密集时（录制每 10 cm 一个点）到达半径只能取很小，偶尔会擦肩而过；
    若还要求绕回该点就会兜圈子。判据：车已越过该点所在的横截面
    （沿路线方向投影 > min_beyond），且横向距离已超过到达半径。"""
    if not robot or not target or not direction: return False
    dx, dy = robot['x'] - target['x'], robot['y'] - target['y']
    length = math.hypot(dx, dy)
    if length <= max(arrival_radius, .02): return False
    nx, ny = float(direction[0]), float(direction[1])
    norm = math.hypot(nx, ny)
    if norm < 1e-9: return False
    return (dx * nx + dy * ny) / norm > min_beyond

def waypoint_index(cursor, mode, total):
    """当前正在驶向的航点序号（界面显示用）。"""
    if total <= 0: return 0
    return cursor % total if mode == 'loop' else min(cursor, total - 1)


def lap_number(cursor, total):
    """按已走过的航点段数推算圈数。"""
    return cursor // total if total > 0 else 0


def prune_reached_goals(goals, pose, radius=.4):
    """去掉开头几个已经在车位容差内的目标。

    Nav2 的连续规划是“从起点依次串到各目标”，若第一个目标与起点重合，
    规划器会直接失败（实测 status=ABORTED、0 路径点）。执行时行为树里的
    RemovePassedGoals 会先剪枝，但预览是裸调用规划动作，需要自己剪。"""
    if pose is None: return list(goals)
    kept = list(goals)
    while kept and math.hypot(kept[0]['x'] - pose['x'], kept[0]['y'] - pose['y']) <= radius:
        kept.pop(0)
    return kept


def start_conflict(meta, grid, pose, margin=.25):
    """车位是否压在障碍或内切膨胀区上。

    这种时候规划器会以“Starting point in lethal space”失败，界面只看到“无可行路径”，
    因此提前判定并给出可执行的提示（重新定位或把车移开）。"""
    if meta is None or grid is None or not pose: return False
    radius = max(1, int(margin / meta['resolution']))
    cx, cy = world_to_cell(meta, pose['x'], pose['y'])
    x0, x1 = max(0, cx - radius), min(meta['width'], cx + radius + 1)
    y0, y1 = max(0, cy - radius), min(meta['height'], cy + radius + 1)
    if x0 >= x1 or y0 >= y1: return False
    return bool(np.any(grid[y0:y1, x0:x1] > 65))


def points_within(points, pose, radius):
    """车位 radius 范围内的雷达点数：用于判断小车是否被障碍贴住（此时规划必失败）。"""
    if not points or not pose: return 0
    r2 = radius * radius
    return sum(1 for p in points if (p[0] - pose['x']) ** 2 + (p[1] - pose['y']) ** 2 <= r2)


def circle_footprint(radius, center=(0., 0.), segments=16):
    """半径为 radius 的圆形有效足迹（多边形近似，用于把障碍周边设为禁区）。"""
    r = max(.05, float(radius)); cx, cy = float(center[0]), float(center[1])
    return [[round(cx + r * math.cos(2 * math.pi * i / segments), 4),
             round(cy + r * math.sin(2 * math.pi * i / segments), 4)] for i in range(segments)]


def footprint_points(footprint):
    """解析足迹，兼容厂商字符串写法 '[ [x, y], ... ]'、嵌套列表与扁平列表。"""
    if isinstance(footprint, str):
        try: footprint = yaml.safe_load(footprint)
        except Exception: return []
    if not isinstance(footprint, (list, tuple)) or not footprint: return []
    if all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in footprint):
        return [[float(footprint[i]), float(footprint[i + 1])] for i in range(0, len(footprint) - 1, 2)]
    out = []
    for item in footprint:
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            out.append([float(item[0]), float(item[1])])
    return out


def footprint_text(points):
    """写回厂商使用的字符串写法，保证 Nav2 解析方式与原来一致。"""
    return '[ ' + ', '.join('[%.4f, %.4f]' % (float(p[0]), float(p[1])) for p in points) + ' ]'


def footprint_center(footprint):
    """足迹多边形中心（用于把圆形禁区套在车体几何中心上）。"""
    pts = footprint_points(footprint)
    if not pts: return (0., 0.)
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    return ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)


def record_step(points, pose, step):
    """航迹记录：距上一个记录点不足 step 米就不记（返回 None），否则返回该点。"""
    if not pose: return None
    x, y = round(float(pose['x']), 3), round(float(pose['y']), 3)
    if points:
        lx, ly = points[-1][0], points[-1][1]
        if math.hypot(x - lx, y - ly) < step - 1e-9: return None   # 恰好等于步长也应记录
    return [x, y]


MAP_SAVE_TOLERANCE = 0.01


def grid_diff_ratio(before, after):
    """两张占据栅格的不同栅格占比（0–1）；形状不同视为完全不同。"""
    if before is None or after is None: return 1.
    if before.shape != after.shape: return 1.
    if before.size == 0: return 0.
    return float(np.count_nonzero(before != after)) / float(before.size)


def should_backup_map(save_current, saved, diff_ratio, tolerance=MAP_SAVE_TOLERANCE):
    """切换地图时是否需要自动备份当前建图。

    save_current: True=强制备份，False=不备份，None=自动判断。
    自动判断下，只要上次保存之后地图没有实质变化（差异 ≤ tolerance）就不再备份、不再打扰用户。"""
    if save_current is False: return False
    if save_current is True: return True
    if not saved: return True
    return diff_ratio > tolerance


MIN_TURN_RADIUS = 0.35


def path_turn_radii(points):
    """相邻三段隐含的转弯半径列表（用于判断路线是否超出车的最小转弯能力）。"""
    radii = []
    for i in range(1, len(points) - 1):
        a = math.atan2(points[i]['y'] - points[i - 1]['y'], points[i]['x'] - points[i - 1]['x'])
        b = math.atan2(points[i + 1]['y'] - points[i]['y'], points[i + 1]['x'] - points[i]['x'])
        turn = abs((b - a + math.pi) % (2 * math.pi) - math.pi)
        length = (math.hypot(points[i]['x'] - points[i - 1]['x'], points[i]['y'] - points[i - 1]['y']) +
                  math.hypot(points[i + 1]['x'] - points[i]['x'], points[i + 1]['y'] - points[i]['y'])) / 2
        if turn > 1e-6 and length > 1e-6: radii.append(length / turn)
    return radii


def path_min_radius(points):
    radii = path_turn_radii(points)
    return min(radii) if radii else None


def infeasible_turn_ratio(points, min_radius=MIN_TURN_RADIUS):
    """转弯比车的最小转弯半径还紧的比例（0–1）。"""
    radii = path_turn_radii(points)
    if not radii: return 0.
    return sum(1 for r in radii if r < min_radius) / len(radii)


def simplify_path(points, epsilon=.12, min_spacing=.25, smooth_passes=2):
    """把遥控录制的原始轨迹整理成可行驶的航点序列。

    录制按固定位移采样（每 10 cm 一个点），叠加定位抖动后局部曲率会远超车的
    最小转弯半径（实测 94 点里 27% 的转弯半径 < 0.35 m，最小 0.07 m），
    规划器无法跟随、只能绕圈。这里先做邻域平滑压掉高频抖动，再用
    Ramer–Douglas–Peucker 抽稀保形，最后按最小间距去重。"""
    pts = [{'x': float(p['x']), 'y': float(p['y'])} for p in points]
    for _ in range(max(0, int(smooth_passes))):
        if len(pts) < 3: break
        smoothed = [pts[0]]
        for i in range(1, len(pts) - 1):
            smoothed.append({'x': (pts[i - 1]['x'] + 2 * pts[i]['x'] + pts[i + 1]['x']) / 4,
                             'y': (pts[i - 1]['y'] + 2 * pts[i]['y'] + pts[i + 1]['y']) / 4})
        smoothed.append(pts[-1]); pts = smoothed
    if len(pts) > 2 and epsilon > 0: pts = _rdp(pts, epsilon)
    result = []
    for p in pts:
        if result and math.hypot(p['x'] - result[-1]['x'], p['y'] - result[-1]['y']) < min_spacing:
            continue
        result.append({'x': round(p['x'], 3), 'y': round(p['y'], 3)})
    if len(result) < 2: result = pts[:2] if len(pts) >= 2 else pts
    return result


def _rdp(points, epsilon):
    """Ramer–Douglas–Peucker 抽稀：只保留偏离直线超过 epsilon 的点。"""
    if len(points) < 3: return list(points)
    start, end = points[0], points[-1]
    dx, dy = end['x'] - start['x'], end['y'] - start['y']
    length = math.hypot(dx, dy)
    worst, index = -1., 0
    for i in range(1, len(points) - 1):
        p = points[i]
        if length < 1e-9:
            dist = math.hypot(p['x'] - start['x'], p['y'] - start['y'])
        else:
            dist = abs(dy * (p['x'] - start['x']) - dx * (p['y'] - start['y'])) / length
        if dist > worst: worst, index = dist, i
    if worst <= epsilon: return [start, end]
    return _rdp(points[:index + 1], epsilon)[:-1] + _rdp(points[index:], epsilon)


RELOCALIZE_MISSION_STATES = ('running', 'accepting', 'paused')


def relocalize_decision(state, now):
    """航行中定位守护的判定，返回 ('wait'|'anchor'|'stop', 原因)。

    吻合度持续偏低说明 AMCL 的位姿与地图对不上：先围绕当前位姿做小窗口吸附重锚；
    连续多次仍不达标就安全停车，避免车在错误位姿下继续走。
    state: enabled / mission / match / low_since / last_at / after_s / interval_s / count / max_count。"""
    if not state.get('enabled'): return 'wait', 'disabled'
    if state.get('mission') not in RELOCALIZE_MISSION_STATES: return 'wait', 'mission'
    match = state.get('match')
    if match is None: return 'wait', 'no_match'
    if match >= state.get('threshold', .55): return 'wait', 'ok'
    low_since = state.get('low_since')
    if low_since is None: return 'wait', 'low'
    if now - low_since < state.get('after_s', 2.): return 'wait', 'low'
    if now - (state.get('last_at') or 0.) < state.get('interval_s', 5.): return 'wait', 'wait'
    if state.get('count', 0) >= state.get('max_count', 4): return 'stop', 'exhausted'
    return 'anchor', 'low'
