# -*- coding: utf-8 -*-
"""问题二连续可靠性裕量、近距离分支与候选选择的回归测试。"""
import json
import unittest
from dataclasses import replace

import numpy as np

from .problem2_solution import (Problem2Config, angular_distance_deg, coverage_margin,
                                direction_result_points, possible_region_samples,
                                reliable_candidate_grid, serialize_result, solve_problem2)


class Problem2Tests(unittest.TestCase):
    """以跨零示向、边界距离和小规模人工工况核验问题二求解器。"""

    def setUp(self):
        """构造快速但仍含边界裁剪的确定性测试网格。"""
        self.config = Problem2Config(angle_step_deg=1.0, radial_step_m=250.0,
                                     candidate_step_m=100.0, boundary_step_m=40.0)

    def test_angular_distance_wraparound(self):
        """0 度两侧的读数应得到小的环形差值。"""
        self.assertAlmostEqual(float(angular_distance_deg(np.array([359.5]), 0.5)[0]), 1.0)

    def test_direction_result_excludes_near_points(self):
        """第二次得到示向度时，五米内候选位置必须由近距离分支处理。"""
        points = np.array([[5.0, 0.0], [5.001, 0.0], [20.0, 0.0]])
        selected = direction_result_points(points, np.zeros(2), 0.0, self.config)
        self.assertEqual(len(selected), 2)
        self.assertTrue(np.all(np.linalg.norm(selected, axis=1) > self.config.near_distance_m))

    def test_reliable_grid_has_conservative_margin(self):
        """所有可靠网格点均满足经覆盖裕量收紧后的边界距离约束。"""
        _, boundary = possible_region_samples(np.zeros(2), 0.0, self.config)
        reliable = reliable_candidate_grid(boundary, self.config)
        self.assertGreater(len(reliable), 0)
        distances = np.linalg.norm(reliable[:, None, :] - boundary[None, :, :], axis=2)
        self.assertLessEqual(float(np.max(distances)),
                             self.config.min_receive_radius_m - coverage_margin(self.config) + 1e-7)

    def test_end_to_end_candidate_selection(self):
        """完整求解应输出可靠点、较优候选点及其中的最终选择。"""
        result = solve_problem2((0.0, 0.0), 359.5, self.config)
        self.assertGreater(len(result.reliable_points), 0)
        self.assertGreater(len(result.good_points), 0)
        self.assertTrue(np.any(np.all(np.isclose(result.good_points, result.selected_point), axis=1)))
        self.assertTrue(np.all(result.quality_values_m >= result.min_quality_m - 1e-9))
        payload = serialize_result(result)
        self.assertEqual(json.loads(json.dumps(payload, ensure_ascii=False))['possible_point_count'],
                         len(result.possible_points))


if __name__ == '__main__':
    unittest.main()
