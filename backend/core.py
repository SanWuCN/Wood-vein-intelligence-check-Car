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
