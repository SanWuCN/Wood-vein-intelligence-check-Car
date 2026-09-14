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


def validate_waypoints(points, meta, grid, mode='multi'):
    if mode not in ('single', 'multi', 'loop'):
        raise ConsoleError('INVALID_MODE', '巡航模式无效', 422)
    if not isinstance(points, list) or not 1 <= len(points) <= 100:
        raise ConsoleError('INVALID_WAYPOINTS', '航点数量应为 1–100', 422)
    if (mode == 'single' and len(points) != 1) or (mode == 'loop' and len(points) < 2):
        raise ConsoleError('INVALID_WAYPOINTS', '航点数量与巡航模式不符', 422)
    if meta is None or grid is None:
        raise ConsoleError('MAP_UNAVAILABLE', '地图未就绪')
    result = [pose_arg(p) for p in points]
    for p in result:
        x, y = world_to_cell(meta, p['x'], p['y'])
        if not (0 <= x < meta['width'] and 0 <= y < meta['height']):
            raise ConsoleError('OUTSIDE_MAP', '航点超出地图', 422)
        v = int(grid[y, x])
        if v < 0 or v > 20:
            raise ConsoleError('POINT_BLOCKED', '航点位于障碍物或未知区域', 422)
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
