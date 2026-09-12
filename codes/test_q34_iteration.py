# -*- coding: utf-8 -*-
"""公共 q3/q4 搜索几何与离线基准核验的性质测试。"""
import math
import unittest

import numpy as np

from .base_models import adaptive_directions, bounded_candidates, open_route_order
from .benchmark_q34_iteration import _check
from .offline_stub import OfflineStub
from .protocol import RobotClient
from .scenario import fixed_scenario


def _route_cost(points, start, order):
    """计算开放路径长度，用于验证 2-opt 不增加代价。"""
    path = [np.asarray(start)] + [np.asarray(points[i]) for i in order]
    return sum(float(np.linalg.norm(path[i] - path[i - 1])) for i in range(1, len(path)))


class GeometryPropertiesTest(unittest.TestCase):
    """验证新增搜索几何函数的输入边界和不变量。"""

    def test_open_route_can_flip_terminal_segment_and_preserves_permutation(self):
        points = np.array([[10., 0.], [11., 10.], [1., 10.], [2., 0.]])
        initial = [0, 1, 2, 3]
        result = open_route_order(points, [0., 0.], initial)
        self.assertEqual(sorted(result), list(range(4)))
        self.assertLessEqual(_route_cost(points, [0., 0.], result),
                             _route_cost(points, [0., 0.], initial) + 1e-7)
        # 开放路径的末端不应人为接回起点；该构造的最优解会翻转末端顺序。
        self.assertEqual(result[-1], 2)

    def test_open_route_rejects_nonfinite_and_bad_order_but_accepts_empty(self):
        self.assertEqual(open_route_order([], [0., 0.]), [])
        with self.assertRaises(ValueError):
            open_route_order([[math.nan, 0.]], [0., 0.])
        with self.assertRaises(ValueError):
            open_route_order([[0., 0.], [1., 0.]], [0., 0.], [0, 0])

    def test_adaptive_directions_rotate_equivariantly(self):
        position, center = np.array([2., -1.]), np.array([8., 3.])
        angle = 0.73
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        first = adaptive_directions(position, center, 8)
        second = adaptive_directions(rotation @ position, rotation @ center, 8)
        np.testing.assert_allclose(second, first @ rotation.T, atol=1e-12)
        np.testing.assert_allclose(adaptive_directions(position, position, 4)[0], [1., 0.])
        with self.assertRaises(ValueError):
            adaptive_directions([math.inf, 0.], [0., 0.], 4)

    def test_bounded_candidates_filters_deduplicates_and_limits(self):
        points = [[0., 0.], [0., 0.000000001], [3., 4.], [6., 0.],
                  [math.inf, 0.], [2_000_001., 0.], [1., 2., 3.]]
        result = bounded_candidates(points, [0., 0.], max_leg_m=5.)
        self.assertEqual(len(result), 2)
        np.testing.assert_allclose(result[0], [0., 0.])
        np.testing.assert_allclose(result[1], [3., 4.])
        self.assertEqual(bounded_candidates([], [0., 0.]), [])
        with self.assertRaises(ValueError):
            bounded_candidates([], [0., 0.], max_leg_m=math.inf)


class BenchmarkValidationTest(unittest.TestCase):
    """确保基准核验能识别轨迹、距离和频道记录被篡改。"""

    def test_check_detects_tampered_distance_and_trajectory(self):
        scenario = fixed_scenario([(1, 0., 0.)])
        stub = OfflineStub(sources=scenario.sources)
        client = RobotClient('offline-team', stub)
        record = {'steps': [{'channel': 1}], 'trajectory': [[0., 0.], [3., 4.]],
                  'moved_distance_m': 99., 'start_virtual_time_s': 0.,
                  'end_virtual_time_s': 1., 'cleared_channels': [1],
                  'planner_errors': 0, 'inconsistent_channels': []}
        checks = _check(record, scenario, client, stub)
        self.assertFalse(checks['distance_match'])
        self.assertFalse(checks['stub_time_match'])
        self.assertFalse(checks['all_constraints_ok'])

    def test_check_detects_duplicate_and_out_of_range_channels(self):
        scenario = fixed_scenario([(1, 0., 0.)])
        stub = OfflineStub(sources=scenario.sources)
        client = RobotClient('offline-team', stub)
        record = {'steps': [], 'trajectory': [[0., 0.]], 'cleared_channels': [1, 1, 21]}
        checks = _check(record, scenario, client, stub)
        self.assertFalse(checks['channels_legal'])
        self.assertFalse(checks['cleared_unique'])
        self.assertFalse(checks['all_constraints_ok'])

    def test_completion_is_derived_for_q3_records_without_completed_field(self):
        scenario = fixed_scenario([(1, 0., 0.)])
        stub = OfflineStub(sources=scenario.sources)
        client = RobotClient('offline-team', stub)
        record = {'steps': [], 'trajectory': [[0., 0.]], 'cleared_channels': [1],
                  'stop_reason': 'all_channels_resolved', 'start_virtual_time_s': 0.,
                  'end_virtual_time_s': 0., 'moved_distance_m': 0.}
        self.assertTrue(_check(record, scenario, client, stub)['complete_valid'])
        record['completed'] = False
        self.assertFalse(_check(record, scenario, client, stub)['complete_valid'])


if __name__ == '__main__':
    unittest.main()
