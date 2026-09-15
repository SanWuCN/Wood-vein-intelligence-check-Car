"""连续巡航的分批规则（纯逻辑，本机可跑）：窗口大小、环状取点、圈数与剩余里程。"""
import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from core import (MAP_SAVE_TOLERANCE, ConsoleError, circle_footprint, footprint_center, footprint_points, footprint_text,
                  grid_diff_ratio, lap_number, plan_batch, points_within, should_backup_map,
                  prune_reached_goals, record_step, remaining_route_distance, start_conflict,
                  validate_waypoints, waypoint_index)


def goals(*xy):
    return [{'x': float(x), 'y': float(y), 'yaw': 0.} for x, y in xy]


class PlanBatchTests(unittest.TestCase):
    def test_multi_sends_all_remaining_by_default(self):
        g = goals((0, 0), (1, 0), (2, 0), (3, 0))
        self.assertEqual(plan_batch(g, 0, 'multi', 0), g)
        self.assertEqual(plan_batch(g, 2, 'multi', 0), g[2:])
        self.assertEqual(plan_batch(g, 4, 'multi', 0), [])

    def test_multi_window_advances_linearly(self):
        g = goals((0, 0), (1, 0), (2, 0), (3, 0), (4, 0))
        self.assertEqual([p['x'] for p in plan_batch(g, 0, 'multi', 2)], [0., 1.])
        self.assertEqual([p['x'] for p in plan_batch(g, 2, 'multi', 2)], [2., 3.])
        self.assertEqual([p['x'] for p in plan_batch(g, 4, 'multi', 2)], [4.])

    def test_single_goal_is_sent_alone(self):
        g = goals((2, 3))
        self.assertEqual(plan_batch(g, 0, 'single', 4), g)

    def test_loop_window_never_covers_whole_lap(self):
        g = goals((0, 0), (1, 0), (1, 1), (0, 1))
        for lookahead in (0, 4, 99):
            batch = plan_batch(g, 0, 'loop', lookahead)
            self.assertLess(len(batch), len(g), '窗口不得覆盖整圈，否则终点会落在车位上')
            self.assertGreaterEqual(len(batch), 1)

    def test_loop_takes_goals_cyclically(self):
        g = goals((0, 0), (1, 0), (1, 1), (0, 1))
        self.assertEqual([p['x'] for p in plan_batch(g, 2, 'loop', 0)], [1., 0., 0.])
        self.assertEqual([p['x'] for p in plan_batch(g, 3, 'loop', 0)], [0., 0., 1.])

    def test_loop_of_two_points_alternates(self):
        g = goals((0, 0), (1, 0))
        self.assertEqual([p['x'] for p in plan_batch(g, 0, 'loop', 0)], [0.])
        self.assertEqual([p['x'] for p in plan_batch(g, 1, 'loop', 0)], [1.])

    def test_waypoint_index_and_lap_number(self):
        self.assertEqual(waypoint_index(0, 'multi', 4), 0)
        self.assertEqual(waypoint_index(9, 'multi', 4), 3)
        self.assertEqual(waypoint_index(4, 'loop', 4), 0)
        self.assertEqual(waypoint_index(5, 'loop', 4), 1)
        self.assertEqual(lap_number(3, 4), 0)
        self.assertEqual(lap_number(4, 4), 1)
        self.assertEqual(lap_number(9, 4), 2)
        self.assertEqual(lap_number(5, 0), 0)


class RemainingDistanceTests(unittest.TestCase):
    def test_multi_adds_following_legs(self):
        g = goals((0, 0), (3, 0), (3, 4), (0, 4))
        # 窗口覆盖 A→B，只剩 1.5 m 到 B；之后 B→C(4) + C→D(3)
        self.assertAlmostEqual(remaining_route_distance(g, 0, 2, 'multi', 1.5), 1.5 + 4.0 + 3.0)
        self.assertAlmostEqual(remaining_route_distance(g, 0, 4, 'multi', 2.0), 2.0)

    def test_loop_counts_rest_of_lap(self):
        g = goals((0, 0), (3, 0), (3, 4), (0, 4))
        # 循环还要算上闭合段 D→A(4)
        self.assertAlmostEqual(remaining_route_distance(g, 0, 2, 'loop', 1.5), 1.5 + 4.0 + 3.0 + 4.0)

    def test_empty_route_is_zero(self):
        self.assertEqual(remaining_route_distance([], 0, 0, 'multi', None), 0.)


class WaypointSpacingTests(unittest.TestCase):
    def setUp(self):
        self.meta = {'width': 40, 'height': 40, 'resolution': .5, 'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
        self.grid = np.zeros((40, 40), np.int16)

    def test_rejects_waypoints_too_close(self):
        # 小于 0.30 m 的相邻航点会让连续导航窗口终点落进到点半径，控制器会立刻判到达
        with self.assertRaises(ConsoleError) as ctx:
            validate_waypoints([{'x': 1., 'y': 1.}, {'x': 1.25, 'y': 1.}], self.meta, self.grid, 'multi')
        self.assertEqual(ctx.exception.code, 'POINTS_TOO_CLOSE')

    def test_rejects_loop_closure_too_close(self):
        with self.assertRaises(ConsoleError) as ctx:
            validate_waypoints([{'x': 1., 'y': 1.}, {'x': 5., 'y': 1.}, {'x': 1.1, 'y': 1.}], self.meta, self.grid, 'loop')
        self.assertEqual(ctx.exception.code, 'POINTS_TOO_CLOSE')

    def test_accepts_reasonable_spacing(self):
        out = validate_waypoints([{'x': 1., 'y': 1.}, {'x': 5., 'y': 1.}, {'x': 5., 'y': 5.}], self.meta, self.grid, 'loop')
        self.assertEqual(len(out), 3)

    def test_threshold_is_0_3m(self):
        # 0.35 m 现在应通过，0.25 m 仍应拒绝
        self.assertEqual(len(validate_waypoints([{'x': 1., 'y': 1.}, {'x': 1.35, 'y': 1.}], self.meta, self.grid, 'multi')), 2)
        with self.assertRaises(ConsoleError):
            validate_waypoints([{'x': 1., 'y': 1.}, {'x': 1.25, 'y': 1.}], self.meta, self.grid, 'multi')


if __name__ == '__main__':
    unittest.main()


class PruneReachedTests(unittest.TestCase):
    def test_drops_leading_goals_at_the_robot(self):
        g = goals((1, 1), (1, 2), (1, 3))
        # 起点与首个目标重合会让规划器直接失败，必须剪掉
        self.assertEqual([p['y'] for p in prune_reached_goals(g, {'x': 1., 'y': 1., 'yaw': 0.}, .4)], [2., 3.])
        self.assertEqual([p['y'] for p in prune_reached_goals(g, {'x': 1.2, 'y': 1., 'yaw': 0.}, .4)], [2., 3.])
        self.assertEqual([p['y'] for p in prune_reached_goals(g, {'x': 1., 'y': .5, 'yaw': 0.}, .4)], [1., 2., 3.])

    def test_keeps_list_when_robot_unknown(self):
        g = goals((1, 1), (1, 2))
        self.assertEqual(prune_reached_goals(g, None), g)


class StartConflictTests(unittest.TestCase):
    def setUp(self):
        self.meta = {'width': 40, 'height': 40, 'resolution': .5, 'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
        self.grid = np.zeros((40, 40), np.int16)
        self.grid[20, 20] = 100          # 10m,10m 处一堵墙

    def test_detects_pose_inside_obstacle(self):
        self.assertTrue(start_conflict(self.meta, self.grid, {'x': 10.1, 'y': 10.1, 'yaw': 0.}))
        self.assertTrue(start_conflict(self.meta, self.grid, {'x': 10.3, 'y': 10., 'yaw': 0.}))   # 内切膨胀范围内

    def test_free_pose_is_clean(self):
        self.assertFalse(start_conflict(self.meta, self.grid, {'x': 5., 'y': 5., 'yaw': 0.}))
        self.assertFalse(start_conflict(self.meta, None, {'x': 10., 'y': 10., 'yaw': 0.}))
        self.assertFalse(start_conflict(self.meta, self.grid, None))


class PointsWithinTests(unittest.TestCase):
    def test_counts_points_near_the_car(self):
        pts = [[1.0, 1.0], [1.3, 1.0], [2.0, 2.0]]
        self.assertEqual(points_within(pts, {'x': 1., 'y': 1., 'yaw': 0.}, .4), 2)
        self.assertEqual(points_within(pts, {'x': 1., 'y': 1., 'yaw': 0.}, .1), 1)
        self.assertEqual(points_within(pts, None, .4), 0)
        self.assertEqual(points_within([], {'x': 1., 'y': 1., 'yaw': 0.}, .4), 0)


class ClearanceTests(unittest.TestCase):
    def test_circle_footprint_is_centered_circle(self):
        fp = circle_footprint(.30, (.089, 0.))
        self.assertEqual(len(fp), 16)
        r = [math.hypot(p[0] - .089, p[1]) for p in fp]
        self.assertAlmostEqual(min(r), .30, places=3)
        self.assertAlmostEqual(max(r), .30, places=3)

    def test_footprint_center_of_ackermann_body(self):
        nested = [[-.031, -.093], [-.031, .093], [.209, .093], [.209, -.093]]
        self.assertEqual(footprint_center(nested), (.089, 0.))
        # 厂商参数里 footprint 是字符串，必须能解析
        self.assertEqual(footprint_center('[ [-0.031, -0.093], [-0.031, 0.093], [0.209, 0.093], [0.209, -0.093] ]'), (.089, 0.))
        self.assertEqual(footprint_center(''), (0., 0.))
        self.assertEqual(footprint_center([]), (0., 0.))

    def test_record_step_only_after_min_distance(self):
        self.assertEqual(record_step([], {'x': 1.2345, 'y': 2.3456, 'yaw': 0.}, .2), [1.234, 2.346])
        pts = [[1.0, 1.0]]
        self.assertIsNone(record_step(pts, {'x': 1.1, 'y': 1.0, 'yaw': 0.}, .2))
        self.assertEqual(record_step(pts, {'x': 1.2, 'y': 1.0, 'yaw': 0.}, .2), [1.2, 1.0])
        self.assertIsNone(record_step(pts, None, .2))


class MapSaveBackupTests(unittest.TestCase):
    """切换地图时是否还要自动备份当前建图。"""

    def test_same_grid_is_not_backed_up_again(self):
        import numpy as np
        g = np.zeros((40, 40), np.int16)
        self.assertEqual(grid_diff_ratio(g, g.copy()), 0.)
        self.assertFalse(should_backup_map(None, True, 0.))
        self.assertFalse(should_backup_map(None, True, MAP_SAVE_TOLERANCE))

    def test_real_changes_trigger_backup(self):
        import numpy as np
        before = np.zeros((40, 40), np.int16)
        after = before.copy(); after[10:20, 10:20] = 100      # 改动 6.25% 栅格
        ratio = grid_diff_ratio(before, after)
        self.assertGreater(ratio, MAP_SAVE_TOLERANCE)
        self.assertTrue(should_backup_map(None, True, ratio))

    def test_explicit_choices_win(self):
        self.assertFalse(should_backup_map(False, False, 1.))   # 明确不保存
        self.assertTrue(should_backup_map(True, True, 0.))      # 明确保存
        self.assertTrue(should_backup_map(None, False, 0.))     # 从没保存过 → 自动备份

    def test_shape_change_counts_as_fully_different(self):
        import numpy as np
        self.assertEqual(grid_diff_ratio(np.zeros((4, 4), np.int16), np.zeros((5, 5), np.int16)), 1.)
        self.assertEqual(grid_diff_ratio(None, np.zeros((4, 4), np.int16)), 1.)

    def test_small_scan_noise_is_tolerated(self):
        import numpy as np
        before = np.zeros((200, 200), np.int16)
        after = before.copy(); after[0, :3] = 100                # 3/40000 = 0.0075%
        self.assertLess(grid_diff_ratio(before, after), MAP_SAVE_TOLERANCE)
        self.assertFalse(should_backup_map(None, True, grid_diff_ratio(before, after)))
