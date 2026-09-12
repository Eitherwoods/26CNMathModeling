# -*- coding: utf-8 -*-
"""DirectionalCoverage 的保守性和输入校验测试。"""
import unittest

import numpy as np

from .problem3_model import Lattice
from .search_evidence import DirectionalCoverage


class DirectionalCoverageTests(unittest.TestCase):
    """验证方向覆盖证据的核心不变量。"""

    def setUp(self):
        self.lattice = Lattice.build(20.0, region_radius_m=80.0)
        self.coverage = DirectionalCoverage(self.lattice)

    def test_mask_and_update_are_idempotent(self):
        mask = self.coverage.new_mask()
        updated = self.coverage.update(mask, (0.0, 0.0))
        np.testing.assert_array_equal(updated, self.coverage.update(updated, (0.0, 0.0)))
        self.assertEqual(self.coverage.gain(updated, (0.0, 0.0)), 0)

    def test_random_source_direction_is_never_excluded(self):
        """对单元内随机真值验证：被清除方向必在站点前半平面且近距内。"""
        rng = np.random.default_rng(7)
        station = np.array([0.0, 0.0])
        evidence = self.coverage.station_mask(station)
        rh = self.lattice.covering_radius_m
        for index, cell in enumerate(self.lattice.points):
            source = cell + rng.uniform(-rh, rh, size=2) * 0.7
            direction = rng.uniform(0.0, 2.0 * np.pi)
            normal = np.array([np.cos(direction), np.sin(direction)])
            bit = int(np.floor((direction % (2.0 * np.pi)) /
                               (2.0 * np.pi / self.coverage.orientation_bins)))
            if evidence[index] & (1 << bit) == 0:
                self.assertLess(np.linalg.norm(station - source), 1000.0)
                self.assertGreater(np.dot(station - cell, normal), 0.0)

    def test_boundary_outward_direction_is_retained(self):
        """边界上朝外方向不得被严格不等式误排。"""
        station = np.array([0.0, 0.0])
        mask = self.coverage.station_mask(station)
        far = int(np.argmax(np.linalg.norm(self.lattice.points, axis=1)))
        offset = station - self.lattice.points[far]
        angles = (np.arange(24) + 0.5) * (2.0 * np.pi / 24)
        outward = int(np.argmin(np.column_stack((np.cos(angles), np.sin(angles))) @ offset))
        self.assertNotEqual(int(mask[far]) & (1 << outward), 0)

    def test_multi_station_ring_reduces_local_evidence(self):
        """多站环可逐步删除中心附近方向证据。"""
        mask = self.coverage.new_mask()
        for station in ((900.0, 0.0), (0.0, 900.0), (-900.0, 0.0), (0.0, -900.0)):
            mask = self.coverage.update(mask, station)
        self.assertLess(int(mask.sum()), int(self.coverage.new_mask().sum()))

    def test_validation(self):
        for bins in (0, 33, 1.5, True):
            with self.assertRaises(ValueError):
                DirectionalCoverage(self.lattice, bins)
        with self.assertRaises(ValueError):
            self.coverage.station_mask((np.nan, 0))
        with self.assertRaises(ValueError):
            self.coverage.update(np.zeros(2, dtype=np.uint32), (0, 0))


if __name__ == '__main__':
    unittest.main()
