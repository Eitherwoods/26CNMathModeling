# -*- coding: utf-8 -*-
"""问题一的解析基准、边界情形及确定性随机几何验证。"""
import unittest
import numpy as np
from scipy.optimize import linprog
from .base_models import locate, solve_halfplanes, bearing_halfplanes
from .problem1_solution import example_data, triangle_data


class GeometryTests(unittest.TestCase):
    """以已知几何和独立线性目标核验顶点的可行性与完整性。"""

    def test_triangle(self):
        """原文三个检测点的交集应精确复现边长十米的正三角形。"""
        observations, truth = triangle_data()
        result = locate(observations)
        expected = np.array([[0, 0], [10, 0], [5, 5 * np.sqrt(3)]])
        self.assertEqual(result.status, 'polygon')
        self.assertEqual(len(result.vertices), 3)
        for point in expected:
            self.assertLess(np.min(np.linalg.norm(result.vertices - point, axis=1)), 1e-7)
        self.assertAlmostEqual(result.diameter, 10)
        midpoint = result.endpoints.mean(axis=0)
        self.assertAlmostEqual(max(np.linalg.norm(result.vertices - midpoint, axis=1)), 5 * np.sqrt(3))
        self.assertAlmostEqual(max(np.linalg.norm(result.vertices - truth, axis=1)), 10 / np.sqrt(3))

    def test_empty_unbounded(self):
        """两个背向锥域为空，单锥域和同向重复观测无界。"""
        self.assertEqual(locate([[0, 0, 180], [10, 0, 0]]).status, 'empty')
        self.assertEqual(locate([[0, 0, 0]]).status, 'unbounded')
        self.assertEqual(locate([[0, 0, 0], [0, 0, 360]]).status, 'unbounded')

    def test_degenerate_and_rectangle(self):
        """解析线段、点、负坐标矩形验证退化分类和自由变量边界。"""
        a = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])
        for b, status, diameter in [([2, 1, 0, 0], 'segment', 3),
                                    ([0, 0, 0, 0], 'point', 0),
                                    ([-1, 4, -2, 6], 'polygon', 5)]:
            result = solve_halfplanes(a, b)
            self.assertEqual(result.status, status)
            self.assertAlmostEqual(result.diameter, diameter)

    def test_wraparound_and_direction(self):
        """叉积包含判定支持跨零度，且排除反向射线。"""
        a, b = bearing_halfplanes([[0, 0, 359.8]])
        self.assertTrue(np.all(a @ [100, 0] <= b))
        self.assertFalse(np.all(a @ [-100, 0] <= b))

    def test_example_and_perturbations(self):
        """三十组有界人工数据以随机线性目标交叉检查全部极值。"""
        observations, truth = example_data()
        rng = np.random.default_rng(20260910)
        for _ in range(30):
            stations = observations[:, :2]
            delta = truth - stations
            angles = np.rad2deg(np.arctan2(delta[:, 1], delta[:, 0]))
            data = np.column_stack((stations, angles + rng.uniform(-1, 1, len(stations))))
            result = locate(data)
            a, b = bearing_halfplanes(data)
            self.assertTrue(np.all(a @ truth <= b + 1e-7))
            self.assertTrue(np.all(a @ result.vertices.T <= b[:, None] + 1e-7))
            for direction in rng.normal(size=(8, 2)):
                reference = linprog(direction, A_ub=a, b_ub=b, bounds=[(None, None)] * 2)
                self.assertTrue(reference.success)
                self.assertAlmostEqual(min(result.vertices @ direction), reference.fun, places=6)
        diameters = [locate(observations, width).diameter for width in (1, 1.2, 1.5)]
        self.assertTrue(np.all(np.diff(diameters) >= -1e-7))

    def test_invalid(self):
        """缺少观测、非有限值和非法角宽应明确报错。"""
        for data in ([], [[0, 0]], [[0, 0, np.nan]]):
            with self.assertRaises(ValueError):
                locate(data)
        with self.assertRaises(ValueError):
            locate([[0, 0, 0]], 90)


if __name__ == '__main__':
    unittest.main()
