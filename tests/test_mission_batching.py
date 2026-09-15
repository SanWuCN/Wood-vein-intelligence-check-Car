"""连续巡航的分批规则（纯逻辑，本机可跑）：窗口大小、环状取点、圈数与剩余里程。"""
import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from core import (MAP_SAVE_TOLERANCE, ConsoleError, circle_footprint, footprint_center, footprint_points, footprint_text,
                  grid_diff_ratio, lap_number, missed_waypoint, plan_batch, points_within, should_backup_map,
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
        self.meta = {'width': 300, 'height': 300, 'resolution': .05, 'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
        self.grid = np.zeros((300, 300), np.int16)

    def test_dense_recorded_route_is_accepted(self):
        # 遥控录制每 10 cm 一个点：必须可以直接用于巡航，不再要求 25/30 cm 间距
        pts = [{'x': 1. + i * .10, 'y': 1.} for i in range(12)]
        self.assertEqual(len(validate_waypoints(pts, self.meta, self.grid, 'multi')), 12)
        self.assertEqual(len(validate_waypoints(pts, self.meta, self.grid, 'loop')), 12)

    def test_only_duplicate_points_are_rejected(self):
        with self.assertRaises(ConsoleError) as ctx:
            validate_waypoints([{'x': 1., 'y': 1.}, {'x': 1.005, 'y': 1.}], self.meta, self.grid, 'multi')
        self.assertEqual(ctx.exception.code, 'POINTS_TOO_CLOSE')
        # 3 cm 已可接受
        self.assertEqual(len(validate_waypoints([{'x': 1., 'y': 1.}, {'x': 1.03, 'y': 1.}], self.meta, self.grid, 'multi')), 2)

    def test_loop_closure_can_be_dense(self):
        pts = [{'x': 1. + i * .1, 'y': 1.} for i in range(4)]
        self.assertEqual(len(validate_waypoints(pts, self.meta, self.grid, 'loop')), 4)


class DenseWindowTests(unittest.TestCase):
    """密集航点：窗口要按距离补足，否则每 4 个点（40 cm）就停一次。"""

    def dense(self, count=40, step=.1):
        return [{'x': i * step, 'y': 0., 'yaw': 0.} for i in range(count)]

    def test_window_covers_min_length(self):
        g = self.dense()
        batch = plan_batch(g, 0, 'multi', 4, min_length_m=2.5, max_points=40)
        self.assertGreaterEqual(len(batch), 25)          # 2.5m / 0.1m ≈ 25 个点
        self.assertLessEqual(len(batch), 40)

    def test_max_points_caps_planning_cost(self):
        g = self.dense(count=200)
        self.assertEqual(len(plan_batch(g, 0, 'multi', 4, min_length_m=10., max_points=30)), 30)

    def test_sparse_route_keeps_point_count_behaviour(self):
        g = [{'x': i * 1.2, 'y': 0., 'yaw': 0.} for i in range(6)]
        self.assertEqual(len(plan_batch(g, 0, 'multi', 4, min_length_m=2.5, max_points=40)), 4)  # 4 点已覆盖 3.6m，无需补足

    def test_loop_dense_window_still_shorter_than_lap(self):
        g = self.dense(count=30)
        batch = plan_batch(g, 0, 'loop', 4, min_length_m=5., max_points=40)
        self.assertLess(len(batch), len(g))              # 一圈 30 点，窗口必须小于一圈
        self.assertEqual(len(batch), 29)                 # 距离补足到 5m 但受“一圈减一”限制

    def test_no_length_constraint_keeps_old_behaviour(self):
        g = self.dense(count=10)
        self.assertEqual(len(plan_batch(g, 0, 'multi', 4)), 4)


class MissedWaypointTests(unittest.TestCase):
    """擦肩而过的航点应跳过，而不是绕回去。"""

    def test_passed_waypoint_is_skipped(self):
        target = {'x': 0., 'y': 0.}
        direction = (1., 0.)                              # 路线沿 +x
        self.assertTrue(missed_waypoint({'x': .2, 'y': .02}, target, direction, .08))   # 已越过
        self.assertTrue(missed_waypoint({'x': .5, 'y': -.3}, target, direction, .08))

    def test_not_yet_reached_is_kept(self):
        target = {'x': 0., 'y': 0.}
        self.assertFalse(missed_waypoint({'x': -.2, 'y': 0.}, target, (1., 0.), .08))    # 还没到
        self.assertFalse(missed_waypoint({'x': .05, 'y': .0}, target, (1., 0.), .08))    # 在到达半径内（会被正常剪枝）

    def test_missing_inputs_are_safe(self):
        self.assertFalse(missed_waypoint(None, {'x': 0., 'y': 0.}, (1., 0.), .08))
        self.assertFalse(missed_waypoint({'x': 1., 'y': 0.}, None, (1., 0.), .08))
        self.assertFalse(missed_waypoint({'x': 1., 'y': 0.}, {'x': 0., 'y': 0.}, (0., 0.), .08))


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
