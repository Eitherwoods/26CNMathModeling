# -*- coding: utf-8 -*-
"""本轮 q3/q4 优化的退化几何与约束回归测试。"""
import unittest
from types import SimpleNamespace

import numpy as np

from .base_models import bearing_intersection
from .problem4_solution import Problem4Config, Problem4Strategy


class BearingIntersectionTests(unittest.TestCase):
    """交会只能返回有限、可辨识的二维点。"""

    def test_parallel_and_missing_readings_have_no_solution(self):
        """平行线不能把最小范数点冒充交会解。"""
        for readings in ([], [[0., 0., 0.]], [[0., 0., 0.], [10., 0., 0.]],
                         [[0., 0., 0.], [0., 10., 180.]],
                         [[0., 0., float('nan')], [0., 10., 90.]]):
            self.assertIsNone(bearing_intersection(readings))

    def test_intersection_is_translation_equivariant(self):
        """大坐标平移仍给出相同相对交点。"""
        readings = np.array([[0., 0., 0.], [100., 100., 270.]])
        np.testing.assert_allclose(bearing_intersection(readings), [100., 0.], atol=1e-10)
        shift = np.array([1e6, -1e6])
        readings[:, :2] += shift
        np.testing.assert_allclose(bearing_intersection(readings), [100., 0.] + shift, atol=1e-8)


class Problem4ProbeTests(unittest.TestCase):
    """q4 交会试探的门控、失败后退路和异常参数。"""

    def setUp(self):
        """只构造公开状态，注入两条相容示向观测。"""
        context = SimpleNamespace(problem=4, state=SimpleNamespace(
            position=(0., 0.), channel=1, virtual_time_s=0.))
        self.strategy = Problem4Strategy(context, Problem4Config(lls_probe=True))
        self.knowledge = self.strategy.channels[1]
        self.knowledge.observe('direction', 0., 0., bearing_deg=0.5)
        self.knowledge.observe('direction', 600., 300., bearing_deg=-45.5)

    def test_probe_is_candidate_and_failure_does_not_mark_cleared(self):
        """生成阶段不修改模型，失败之后拒绝重复下注且保留后备点。"""
        before = self.knowledge.possible_points()
        plan = self.strategy._tracking_plan(1)
        self.assertEqual(plan.kind, 'lls_probe')
        np.testing.assert_array_equal(before, self.knowledge.possible_points())
        self.knowledge.observe_clear_failure(*plan.waypoint)
        self.assertEqual(self.knowledge.status, 'detected')
        self.assertIsNone(self.strategy._lls_probe_point(self.knowledge))
        self.assertIsNotNone(self.knowledge.fallback_point(np.zeros(2)))

    def test_probe_disabled_and_invalid_configuration(self):
        """开关关闭保持旧路径，异常参数在构造时被拒绝。"""
        self.strategy.config = Problem4Config(lls_probe=False)
        self.assertIsNone(self.strategy._lls_probe_point(self.knowledge))
        for values in ({'lls_probe': 1}, {'lls_probe_min_reads': True},
                       {'lls_probe_min_reads': 1}, {'lls_probe_max_mask_dist_m': float('nan')},
                       {'lls_probe_min_separation_m': 19.}):
            with self.assertRaises(ValueError):
                Problem4Config(**values)


if __name__ == '__main__':
    unittest.main()
