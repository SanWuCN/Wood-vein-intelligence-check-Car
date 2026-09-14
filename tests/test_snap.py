"""人工定位吸附微调：用合成房间 + 射线投射的假雷达验证算法确实能纠正偏差。"""
import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from core import grid_likelihood, nearest_free, pose_score, snap_pose


def room_grid(width=200, height=200, res=.05, origin=(-5., -5., 0.), room=(-3., -2.5, 3., 2.5)):
    """返回 (meta, grid)：room 为 (x0,y0,x1,y1) 的矩形房间，墙为占据、室内空闲、室外未知。"""
    grid = np.full((height, width), -1, np.int16)
    x0, y0, x1, y1 = room
    for j in range(height):
        for i in range(width):
            x = origin[0] + (i + .5) * res
            y = origin[1] + (j + .5) * res
            if not (x0 <= x <= x1 and y0 <= y <= y1): continue
            on_wall = x < x0 + res or x > x1 - res or y < y0 + res or y > y1 - res
            grid[j, i] = 100 if on_wall else 0
    meta = {'width': width, 'height': height, 'resolution': res,
            'origin': {'x': origin[0], 'y': origin[1], 'yaw': origin[2]},
            'frame_id': 'map', 'bounds': [0, 0, width, height]}
    return meta, grid


def scan_from(meta, grid, pose, rays=240, rmin=.15, rmax=6.):
    """在真实位姿上模拟一圈激光，返回 base 系下的点（含少量噪声，模拟真实雷达）。"""
    res = meta['resolution']
    ox, oy = meta['origin']['x'], meta['origin']['y']
    pts = []
    for k in range(rays):
        ang = -math.pi + 2 * math.pi * k / rays
        r = rmin
        while r < rmax:
            wx, wy = pose['x'] + r * math.cos(pose['yaw'] + ang), pose['y'] + r * math.sin(pose['yaw'] + ang)
            i, j = int((wx - ox) / res), int((wy - oy) / res)
            if not (0 <= i < meta['width'] and 0 <= j < meta['height']): break
            if grid[j, i] > 20: break
            r += res * .5
        else:
            continue
        if r >= rmax: continue
        pts.append((r * math.cos(ang), r * math.sin(ang)))
    return np.asarray(pts, dtype=np.float32)


class SnapTests(unittest.TestCase):
    def setUp(self):
        self.meta, self.grid = room_grid()
        self.truth = {'x': .82, 'y': -.37, 'yaw': math.radians(21.)}
        self.scan = scan_from(self.meta, self.grid, self.truth)

    def test_likelihood_peaks_on_obstacle(self):
        like = grid_likelihood(self.grid, self.meta['resolution'])
        self.assertEqual(like.shape, self.grid.shape)
        self.assertAlmostEqual(float(like.max()), 1.0, places=5)
        # 室内中心离墙远，分数应低于贴墙处
        cx, cy = int((0 - self.meta['origin']['x']) / .05), int((0 - self.meta['origin']['y']) / .05)
        wall = int((-3.0 - self.meta['origin']['x']) / .05) + 1
        self.assertLess(like[cy, cx], like[cy, wall])

    def test_snap_recovers_offset_pose(self):
        guess = {'x': self.truth['x'] + .13, 'y': self.truth['y'] - .11, 'yaw': self.truth['yaw'] + math.radians(6.)}
        like = grid_likelihood(self.grid, self.meta['resolution'])
        before = pose_score(like, self.meta, self.scan, guess['x'], guess['y'], guess['yaw'])
        out = snap_pose(self.meta, self.grid, self.scan, guess, like=like)
        self.assertTrue(out['applied'])
        self.assertGreater(out['score'], before)
        self.assertLess(math.hypot(out['pose']['x'] - self.truth['x'], out['pose']['y'] - self.truth['y']), .06)
        self.assertLess(abs(math.degrees(out['pose']['yaw'] - self.truth['yaw'])), 3.)
        self.assertGreater(out['shift_m'], 0.05)          # 确实做了微调
        self.assertLessEqual(out['shift_m'], .5)          # 但幅度受限

    def test_snap_keeps_good_pose(self):
        like = grid_likelihood(self.grid, self.meta['resolution'])
        out = snap_pose(self.meta, self.grid, self.scan, dict(self.truth), like=like)
        self.assertLess(math.hypot(out['pose']['x'] - self.truth['x'], out['pose']['y'] - self.truth['y']), .06)
        self.assertLess(out['shift_m'], .08)

    def test_snap_moves_out_of_obstacle(self):
        blocked = {'x': -3.0, 'y': 0.0, 'yaw': 0.}      # 正好点在墙上
        out = snap_pose(self.meta, self.grid, None, blocked)
        self.assertGreater(out['free_shift_m'], 0.)
        self.assertEqual(out['reason'], 'no_scan')
        i, j = int((out['pose']['x'] - self.meta['origin']['x']) / .05), int((out['pose']['y'] - self.meta['origin']['y']) / .05)
        self.assertTrue(0 <= int(self.grid[j, i]) <= 20)

    def test_snap_reports_missing_inputs(self):
        out = snap_pose(self.meta, self.grid, None, {'x': 0., 'y': 0., 'yaw': 0.})
        self.assertEqual(out['reason'], 'no_scan')
        self.assertIsNone(out['score'])
        out = snap_pose(None, None, self.scan, {'x': 0., 'y': 0., 'yaw': 0.})
        self.assertEqual(out['reason'], 'no_map')

    def test_snap_refuses_when_map_matches_poorly(self):
        # 扫描与地图完全无关（随机点）：判据不足，必须保持用户点选位置
        rng = np.random.default_rng(3)
        junk = rng.uniform(-4, 4, (160, 2)).astype(np.float32)
        guess = {'x': 0., 'y': 0., 'yaw': 0.}
        out = snap_pose(self.meta, self.grid, junk, guess)
        self.assertEqual(out['shift_m'], 0.0)
        self.assertEqual(out['shift_deg'], 0.0)
        self.assertIn(out['reason'], ('weak_match', None))

    def test_nearest_free_ignores_free_point(self):
        self.assertIsNone(nearest_free(self.meta, self.grid, 0., 0.))
        # 地图范围内但房间外（未知区）：吸附到最近的室内空闲格
        out = nearest_free(self.meta, self.grid, 4.0, 4.0)
        self.assertIsNotNone(out)
        self.assertGreater(out[0], 1.5)
        # 远在地图之外：搜索有上限，返回 None（随后由地图范围校验拒绝）
        self.assertIsNone(nearest_free(self.meta, self.grid, 40.0, 40.0))


if __name__ == '__main__':
    unittest.main()
