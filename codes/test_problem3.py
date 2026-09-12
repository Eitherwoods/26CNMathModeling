# -*- coding: utf-8 -*-
"""问题三栅格模型、覆盖式不存在性证明、清除判据与离线端到端回归测试。

本文件只使用本地桩（`OfflineStub`），不连接官方模拟器。
按 `tester/README.md` 的约定，演练测试与正式测试一律由人工在模拟器界面触发。
"""
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

from . import config as settings
from .base_models import distance_to_wedge, min_enclosing_circle, sampled_diameter
from .config import (CHANNEL_COUNT, CLEAR_RADIUS_M, MAX_RECEIVE_RADIUS_M,
                     MIN_RECEIVE_RADIUS_M, MIN_SOURCE_COUNT, NEAR_DISTANCE_M,
                     ROBOT_SPEED_MPS, TARGET_RADIUS_M)
from .offline_stub import OfflineStub, Source
from .problem3_model import (ChannelKnowledge, Lattice, apply_direction,
                             apply_distance_order, apply_near, apply_outside, apply_wedge,
                             coverage_radius_m, hex_waypoints,
                             sweep_waypoint_gain, wedge_distance)
from .problem3_solution import (Problem3Config, Problem3Strategy, StopPlan,
                                is_offline_run, run_mission, solve, summarize)
from .protocol import RobotClient, decode, encode
from .scenario import fixed_scenario, random_scenario, sweep_waypoints
from .strategy import StrategyContext, run_strategy

PROBE = np.array([[300.0, 4.0], [1000.0, 0.0], [300.0, 30.0], [3.0, 0.0],
                  [1600.0, 0.0], [-300.0, 0.0], [0.0, 40.0], [40.0, 0.0],
                  [10.0, 0.0], [14.0, 0.0], [20.0, 0.0], [18.0, 0.0],
                  [25.0, 0.0], [0.0, 0.0]])


def probe_lattice(spacing=1.0):
    """把探测点直接当作格网点，使掩码断言可以与坐标精确对应。"""
    return Lattice(PROBE, float(spacing), 1601.0, float(spacing) * np.sqrt(2.0) / 2.0)


def survivors(mask):
    """返回掩码保留的格点坐标集合。"""
    return {tuple(point) for point, keep in zip(PROBE, mask) if keep}


class LatticeTests(unittest.TestCase):
    """格网构造：覆盖半径、区域包含性与非法输入。"""

    def test_covering_radius_matches_spacing(self):
        """覆盖半径必须为 h*sqrt(2)/2，且格网覆盖整个目标圆域。"""
        lattice = Lattice.build(10.0)
        self.assertAlmostEqual(lattice.covering_radius_m, 10.0 * np.sqrt(2.0) / 2.0, places=9)
        radius = np.hypot(lattice.points[:, 0], lattice.points[:, 1])
        self.assertLessEqual(float(radius.max()), lattice.extent_m + 1e-9)
        self.assertGreaterEqual(lattice.extent_m, TARGET_RADIUS_M)

    def test_covering_radius_bounds_true_distance(self):
        """任意目标点都有格点在覆盖半径内代表它（掩码严格性的基础）。"""
        lattice = Lattice.build(25.0)
        rng = np.random.default_rng(0)
        angle = rng.uniform(0.0, 2.0 * np.pi, 400)
        distance = TARGET_RADIUS_M * np.sqrt(rng.uniform(0.0, 1.0, 400))
        queries = np.column_stack((distance * np.cos(angle), distance * np.sin(angle)))
        nearest = np.linalg.norm(queries[:, None, :] - lattice.points[None, :, :],
                                 axis=2).min(axis=1)
        self.assertLessEqual(float(nearest.max()), lattice.covering_radius_m + 1e-9)

    def test_invalid_spacing_rejected(self):
        """步长必须为正的有限值。"""
        for spacing in (0.0, -1.0, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                Lattice.build(spacing)


class GeometryTests(unittest.TestCase):
    """公共几何：角域距离、最小覆盖圆与直径的关系。"""

    def test_wedge_distance_zero_inside_positive_outside(self):
        """角域内距离为 0，角域外为正且随偏离角增大；与公共几何函数一致。"""
        offset = np.array([[100.0, 0.0], [100.0, 1.0], [100.0, 20.0], [-100.0, 0.0]])
        values = wedge_distance(offset, 0.0)
        self.assertAlmostEqual(float(values[0]), 0.0)
        self.assertAlmostEqual(float(values[1]), 0.0)   # 0.57 度仍在 ±1 度内
        self.assertGreater(float(values[2]), 0.0)
        self.assertGreater(float(values[3]), 0.0)       # 反向不在锥内
        reference = distance_to_wedge(np.array([[100.0, 20.0]]), (0.0, 0.0), 0.0, 1.0)
        self.assertAlmostEqual(float(values[2]), float(reference[0]), places=9)

    def test_min_enclosing_circle_on_right_triangle(self):
        """三点最小覆盖圆回到外接圆（直角三角形：圆心在斜边中点）。"""
        points = np.array([[0.0, 0.0], [60.0, 0.0], [0.0, 80.0]])
        center, radius = min_enclosing_circle(points)
        self.assertTrue(np.allclose(center, [30.0, 40.0], atol=1e-6))
        self.assertAlmostEqual(radius, 50.0, places=6)

    def test_covering_circle_obeys_jung_bounds(self):
        """覆盖圆半径受 Jung 定理约束：diam/2 <= r <= diam/sqrt(3)，两点相等只在特殊构型。"""
        spread = np.array([[0.0, 0.0], [100.0, 0.0], [50.0, 86.6025403784]])
        _, radius = min_enclosing_circle(spread)
        diameter = sampled_diameter(spread)
        self.assertAlmostEqual(radius, diameter / np.sqrt(3.0), places=6)
        self.assertGreater(radius, diameter / 2.0)
        obtuse = np.array([[0.0, 0.0], [100.0, 0.0], [100.0, 1.0]])
        _, radius = min_enclosing_circle(obtuse)
        self.assertAlmostEqual(radius, sampled_diameter(obtuse) / 2.0, places=6)
        # 直径圆半径等于 diam，至少比真实覆盖圆大 1.73 倍，直接沿用会过度保守。
        self.assertGreater(diameter, radius)

    def test_min_enclosing_circle_on_wedge_matches_symmetry(self):
        """楔形掩码的覆盖圆必须落在楔形对称轴上（旧版迭代解法在此偏大 18%）。"""
        lattice = Lattice.build(10.0)
        knowledge = ChannelKnowledge(1, lattice, lattice)
        knowledge.observe_direction(0.0, 0.0, 0.0)
        center, radius = knowledge.region_estimate(use_fine=True)
        self.assertLessEqual(abs(float(center[1])), 2.0)
        self.assertLess(radius, 790.0)
        points = lattice.points[knowledge.mask]
        self.assertLessEqual(float(np.max(np.linalg.norm(points - center, axis=1))), radius + 1e-6)


class MaskUpdateTests(unittest.TestCase):
    """保守更新算子：交类与差类更新的偏离方向都必须朝"不误删"一侧。"""

    def setUp(self):
        """探测点格网，覆盖半径 0.707 米，便于分辨 5 米与 20 米边界。"""
        self.lattice = probe_lattice()

    def test_direction_keeps_wedge_and_annulus(self):
        """示向度更新保留角域内、距离落在 (5, 1500] 内的格点。"""
        mask = apply_direction(np.ones(len(PROBE), dtype=bool), self.lattice, 0.0, 0.0, 0.0)
        kept = survivors(mask)
        self.assertIn((300.0, 4.0), kept)        # 偏离 0.76 度，仍在角域内
        self.assertIn((1000.0, 0.0), kept)
        self.assertNotIn((300.0, 30.0), kept)    # 偏离 5.7 度
        self.assertNotIn((3.0, 0.0), kept)       # 太近
        self.assertNotIn((1600.0, 0.0), kept)    # 超出最大接收半径
        self.assertNotIn((-300.0, 0.0), kept)    # 反方向
        self.assertNotIn((0.0, 40.0), kept)      # 偏离 90 度

    def test_wedge_rotates_with_bearing(self):
        """示向角域随示向度旋转：90 度时保留北向格点。"""
        mask = apply_wedge(np.ones(len(PROBE), dtype=bool), self.lattice, 0.0, 0.0, 90.0)
        kept = survivors(mask)
        self.assertIn((0.0, 40.0), kept)
        self.assertNotIn((40.0, 0.0), kept)

    def test_near_shrinks_to_five_metre_ball(self):
        """距离过近把位置集合收缩到五米圆域（按覆盖半径外扩）。"""
        mask = apply_near(np.ones(len(PROBE), dtype=bool), self.lattice, 10.0, 0.0)
        kept = survivors(mask)
        self.assertIn((10.0, 0.0), kept)
        self.assertIn((14.0, 0.0), kept)         # 4 米 < 5 + 0.707
        self.assertNotIn((20.0, 0.0), kept)
        self.assertNotIn((0.0, 0.0), kept)

    def test_outside_deletes_only_provably_impossible_points(self):
        """排除类更新只删除覆盖半径邻域完全落入被排除圆域的格点。"""
        mask = apply_outside(np.ones(len(PROBE), dtype=bool), self.lattice, 0.0, 0.0, 20.0)
        kept = survivors(mask)
        self.assertNotIn((18.0, 0.0), kept)      # 18 < 20 - 0.707，确实不可能
        self.assertIn((20.0, 0.0), kept)         # 边界点保留，不冒险删
        self.assertIn((25.0, 0.0), kept)
        self.assertAlmostEqual(20.0 - self.lattice.covering_radius_m,
                               20.0 - np.sqrt(2.0) / 2.0, places=9)


class ChannelKnowledgeTests(unittest.TestCase):
    """单频道判据：距离上界、可证清除、均匀先验概率与最小覆盖圆。"""

    def setUp(self):
        """一米步长、半径 50 米的小格网，细粗两档合用。"""
        self.lattice = Lattice.build(1.0, region_radius_m=50.0)
        self.knowledge = ChannelKnowledge(7, self.lattice, self.lattice)

    def test_fresh_channel_is_active_with_full_area(self):
        """初始频道未被排除，面积等于整个格网。"""
        self.assertTrue(self.knowledge.is_active)
        self.assertFalse(self.knowledge.is_excluded)
        self.assertAlmostEqual(self.knowledge.possible_area_m2, len(self.lattice), places=6)

    def test_near_observation_makes_clear_certain(self):
        """五米内读数后，距离上界含覆盖半径仍不超过 20 米。"""
        self.knowledge.observe_near(0.0, 0.0)
        limit = self.knowledge.max_distance_m(0.0, 0.0)
        self.assertIsNotNone(limit)
        self.assertLessEqual(limit,
                             NEAR_DISTANCE_M + 2.0 * self.lattice.covering_radius_m + 1e-9)
        self.assertTrue(self.knowledge.certain_clear(0.0, 0.0))
        self.assertTrue(self.knowledge.near_certain_clear(0.0, 0.0))
        self.assertFalse(self.knowledge.near_certain_clear(10.0, 0.0))
        self.assertAlmostEqual(self.knowledge.clear_probability(0.0, 0.0), 1.0, places=6)

    def test_direction_only_channel_is_not_certain(self):
        """仅有示向度时距离上界很大，不得判为可证清除。"""
        self.knowledge.observe_direction(0.0, 0.0, 0.0)
        limit = self.knowledge.max_distance_m(0.0, 0.0)
        self.assertGreater(limit, CLEAR_RADIUS_M)
        self.assertFalse(self.knowledge.certain_clear(0.0, 0.0))
        self.assertLessEqual(limit,
                             MAX_RECEIVE_RADIUS_M + 2.0 * self.lattice.covering_radius_m + 1e-9)

    def test_direction_observation_shrinks_to_narrow_wedge(self):
        """示向度更新后可能位置集合应显著收缩，且集中在示向方向上。"""
        before = self.knowledge.possible_area_m2
        self.knowledge.observe_direction(0.0, 0.0, 0.0)
        after = self.knowledge.possible_area_m2
        self.assertLess(after, before)
        center, radius = self.knowledge.region_estimate(use_fine=True)
        self.assertLessEqual(radius, 50.0)
        self.assertGreater(float(center[0]), 0.0)

    def test_clear_failure_excludes_clear_radius(self):
        """清除失败等价于"目标不在 20 米内"，据此删除该圆域内的格点。"""
        self.knowledge.observe_direction(0.0, 0.0, 0.0)
        before = self.knowledge.possible_area_m2
        self.knowledge.observe_clear_failure(0.0, 0.0)
        self.assertLess(self.knowledge.possible_area_m2, before)
        self.assertTrue(self.knowledge.cleared_here(0.0, 0.0))
        self.assertFalse(self.knowledge.cleared_here(5.0, 0.0))
        self.assertTrue(self.knowledge.is_active)

    def test_clear_failure_removes_inner_disc_only(self):
        """清除失败的排除圆半径是 20 米（按覆盖半径保守收紧）。"""
        lattice = probe_lattice()
        knowledge = ChannelKnowledge(7, lattice, lattice)
        knowledge.observe_clear_failure(0.0, 0.0)
        kept = survivors(knowledge.mask)
        self.assertNotIn((18.0, 0.0), kept)
        self.assertIn((20.0, 0.0), kept)
        self.assertIn((25.0, 0.0), kept)

    def test_near_then_clear_failure_is_flagged_inconsistent(self):
        """先"距离过近"(<=5 m)再"清除失败"(>20 m)互相矛盾，必须报矛盾而非判不存在。"""
        self.knowledge.observe_near(0.0, 0.0)
        self.knowledge.observe_clear_failure(0.0, 0.0)
        self.assertTrue(self.knowledge.is_inconsistent)
        self.assertFalse(self.knowledge.is_excluded)
        self.assertTrue(self.knowledge.is_active)
        self.assertTrue(self.knowledge.cleared_here(0.0, 0.0))

    def test_mark_cleared_empties_and_deactivates(self):
        """清除成功后位置集合清空、状态转为已清除。"""
        self.knowledge.mark_cleared(123.0)
        self.assertEqual(self.knowledge.status, 'cleared')
        self.assertEqual(self.knowledge.cleared_virtual_s, 123.0)
        self.assertTrue(self.knowledge.is_excluded)
        self.assertFalse(self.knowledge.is_active)
        self.assertAlmostEqual(self.knowledge.possible_area_m2, 0.0)

    def test_repeated_measurement_at_same_point_is_suppressed(self):
        """同点重复检测必须被识别（同点误差固定，重复读数不含新信息）。"""
        self.knowledge.observe_direction(100.0, 0.0, 45.0)
        self.assertIsNotNone(self.knowledge.measured_at(100.0, 0.0))
        self.assertIsNone(self.knowledge.measured_at(100.5, 0.0))

    def test_near_first_reading_marks_channel_detected(self):
        """首个读数就是距离过近时，频道必须进入已发现状态。

        否则清除候选（按已发现频道筛选）会整个跳过它，导致该源被永久漏掉。
        """
        self.knowledge.observe_near(0.0, 0.0)
        self.assertEqual(self.knowledge.status, 'detected')
        self.assertTrue(self.knowledge.is_active)
        self.assertFalse(self.knowledge.is_excluded)
        self.assertIsNotNone(self.knowledge.detected_virtual_s)
        self.assertTrue(self.knowledge.certain_clear(0.0, 0.0))

    def test_no_signal_alone_does_not_mark_channel_detected(self):
        """无信号读数不构成"已发现"：频道仍是未知状态。"""
        lattice = probe_lattice()
        knowledge = ChannelKnowledge(7, lattice, lattice)
        knowledge.observe_no_signal(0.0, 0.0)
        self.assertEqual(knowledge.status, 'unknown')
        self.assertEqual(len(knowledge.observations), 1)
        self.assertTrue(knowledge.is_active)
        self.assertFalse(knowledge.is_excluded)
        self.assertNotIn((18.0, 0.0), survivors(knowledge.mask))
        self.assertIn((1600.0, 0.0), survivors(knowledge.mask))

    def test_distance_order_shrinks_after_signal_and_no_signal(self):
        """共同未知接收半径应删除明显更靠近无信号点的候选。"""
        lattice = probe_lattice()
        mask = apply_distance_order(np.ones(len(PROBE), dtype=bool), lattice,
                                    [(0.0, 0.0)], [(100.0, 0.0)])
        self.assertIn((40.0, 0.0), survivors(mask))
        self.assertNotIn((1000.0, 0.0), survivors(mask))

    def test_distance_order_is_order_independent(self):
        """先收信后无信号与先无信号后收信得到同一保守结果。"""
        lattice = probe_lattice()
        first = ChannelKnowledge(7, lattice, lattice)
        second = ChannelKnowledge(7, lattice, lattice)
        first.observe_direction(0.0, 0.0, 0.0)
        first.observe_no_signal(100.0, 0.0)
        second.observe_no_signal(100.0, 0.0)
        second.observe_direction(0.0, 0.0, 0.0)
        self.assertTrue(np.array_equal(first.mask, second.mask))
        self.assertTrue(np.array_equal(first.plan_mask, second.plan_mask))

    def test_distance_order_keeps_boundary_for_conservative_safety(self):
        """边界及不确定性带内的点必须保留，不能因严格比较误删真值。"""
        lattice = probe_lattice()
        mask = apply_distance_order(np.ones(len(PROBE), dtype=bool), lattice,
                                    [(0.0, 0.0)], [(40.0, 0.0)])
        self.assertIn((20.0, 0.0), survivors(mask))
        self.assertNotIn((40.0, 0.0), survivors(mask))

    def test_detected_channel_with_empty_mask_is_inconsistent(self):
        """已取得读数却算出空掩码属矛盾状态：不得当成"该频道不存在"。"""
        self.knowledge.observe_direction(0.0, 0.0, 0.0)
        self.assertFalse(self.knowledge.is_inconsistent)
        self.knowledge.mask[:] = False
        self.assertTrue(self.knowledge.is_inconsistent)
        self.assertFalse(self.knowledge.is_excluded)
        self.assertTrue(self.knowledge.is_active)

    def test_cleared_channel_is_not_inconsistent(self):
        """正常清除产生的空掩码不是矛盾状态。"""
        self.knowledge.mark_cleared(10.0)
        self.assertFalse(self.knowledge.is_inconsistent)
        self.assertFalse(self.knowledge.is_active)

    def test_region_estimate_excludes_covering_radius(self):
        """最小覆盖圆只返回几何外接半径：至多为 5 + r_h，不含第二次覆盖半径外扩。"""
        self.knowledge.observe_near(0.0, 0.0)
        center, radius = self.knowledge.region_estimate(use_fine=True)
        covering = self.lattice.covering_radius_m
        self.assertLessEqual(float(np.hypot(*center)), 1.0)
        self.assertLessEqual(radius, NEAR_DISTANCE_M + covering + 1e-9)
        self.assertGreaterEqual(radius, NEAR_DISTANCE_M)
        self.assertLess(radius, NEAR_DISTANCE_M + 2.0 * covering)


class CoverageProofTests(unittest.TestCase):
    """覆盖式不存在性证明：七点布站的完备性、下界与代价口径。"""

    def setUp(self):
        """20 米格网足以支撑覆盖判据，同时保持测试快速。"""
        self.lattice = Lattice.build(20.0)
        self.budget = MIN_RECEIVE_RADIUS_M - self.lattice.covering_radius_m

    def test_seven_station_ring_proves_absence(self):
        """七个停靠点的无信号读数必须把该频道在目标区域内的可能位置清空。"""
        waypoints = hex_waypoints(1200.0)
        self.assertEqual(len(waypoints), 7)
        self.assertLessEqual(coverage_radius_m(self.lattice.points, waypoints), self.budget)
        knowledge = ChannelKnowledge(3, self.lattice, self.lattice)
        for x, y in waypoints:
            self.assertFalse(knowledge.is_excluded)
            knowledge.observe_no_signal(x, y)
        self.assertTrue(knowledge.is_excluded)

    def test_single_station_cannot_prove_absence(self):
        """单点无信号只排除半径 1000 的圆域，远不足以证明整个区域不存在。"""
        knowledge = ChannelKnowledge(4, self.lattice, self.lattice)
        knowledge.observe_no_signal(0.0, 0.0)
        self.assertFalse(knowledge.is_excluded)
        self.assertGreater(knowledge.possible_area_m2, 0.0)

    def test_ring_too_small_leaves_gap(self):
        """环半径过小时环内出现漏检，覆盖判据必须失效。"""
        self.assertGreater(coverage_radius_m(self.lattice.points, hex_waypoints(1000.0)),
                           self.budget)

    def test_feasible_ring_band_and_minimum_station_count(self):
        """环半径可行区间下界约 1123 米；环点数少于 6 时边界漏检不可避免。"""
        def leak(radius):
            return coverage_radius_m(self.lattice.points, hex_waypoints(radius))
        self.assertGreater(leak(1100.0), self.budget)
        self.assertLessEqual(leak(1200.0), self.budget)
        self.assertLessEqual(leak(1558.85), self.budget)
        self.assertLess(leak(1558.85), leak(1200.0))
        self.assertGreater(coverage_radius_m(self.lattice.points, hex_waypoints(1800.0)),
                           self.budget)

    def test_sweep_gain_zero_when_channel_already_excluded(self):
        """已排除频道在任何停靠点都不再有新增排除面积。"""
        knowledge = ChannelKnowledge(5, self.lattice, self.lattice)
        for x, y in hex_waypoints(1200.0):
            knowledge.observe_no_signal(x, y)
        self.assertTrue(knowledge.is_excluded)
        gain = sweep_waypoint_gain(np.array([[0.0, 0.0]]), [knowledge.mask],
                                   self.lattice, MIN_RECEIVE_RADIUS_M)
        self.assertAlmostEqual(float(gain[0]), 0.0)

    def test_sweep_gain_positive_for_unresolved_channel(self):
        """未排除频道在停靠点上有正的粗格网新增排除面积。"""
        knowledge = ChannelKnowledge(6, self.lattice, self.lattice)
        gain = sweep_waypoint_gain(np.array([[0.0, 0.0]]), [knowledge.mask],
                                   self.lattice, MIN_RECEIVE_RADIUS_M)
        self.assertGreater(float(gain[0]), 0.0)

    def test_scenario_sweep_waypoints_match_model(self):
        """场景模块的布站接口与模型层保持一致。"""
        self.assertTrue(np.allclose(sweep_waypoints(1200.0), hex_waypoints(1200.0)))


class ScenarioAndStubTests(unittest.TestCase):
    """离线案例生成与本地桩语义（不是官方模拟器的案例分布）。"""

    def test_random_scenario_respects_constraints(self):
        """频道互不相同、位置落在目标圆域内、间距与接收半径合规。"""
        scenario = random_scenario(seed=3, n_sources=10)
        self.assertEqual(len(scenario.sources), 10)
        channels = [source.channel for source in scenario.sources]
        self.assertEqual(len(set(channels)), 10)
        self.assertTrue(all(1 <= channel <= 20 for channel in channels))
        positions = np.array([[source.x, source.y] for source in scenario.sources])
        self.assertLessEqual(float(np.linalg.norm(positions, axis=1).max()), TARGET_RADIUS_M)
        pairwise = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2)
        self.assertGreaterEqual(float(pairwise[~np.eye(len(positions), dtype=bool)].min()), 80.0)
        self.assertTrue(all(1000.0 <= source.radius <= 1500.0 for source in scenario.sources))

    def test_random_scenario_is_reproducible(self):
        """同种子必须给出同案例。"""
        first = random_scenario(seed=11, n_sources=6)
        second = random_scenario(seed=11, n_sources=6)
        self.assertEqual([(s.channel, s.x, s.y) for s in first.sources],
                         [(s.channel, s.x, s.y) for s in second.sources])

    def test_stub_result_branches(self):
        """本地桩的三类检测返回与清除判定符合附录 2 的距离口径。"""
        stub = OfflineStub(sources=[Source(channel=2, x=100.0, y=0.0, radius=1000.0)])
        client = RobotClient('offline-team', stub)
        client.enter()
        self.assertEqual(client.measure(0.0, 0.0, 2)['measure_result'], 'direction')
        self.assertAlmostEqual(client.measure(0.0, 0.0, 2)['svd_deg'], 0.0, places=6)
        self.assertEqual(client.measure(97.0, 0.0, 2)['measure_result'], 'near')
        self.assertEqual(client.measure(1500.0, 0.0, 2)['measure_result'], 'no_signal')
        self.assertEqual(client.clear(110.0, 0.0, 2)['clear_result'], 'success')
        self.assertEqual(client.measure(110.0, 0.0, 2)['measure_result'], 'no_signal')
        client.exit()

    def test_stub_location_error_is_fixed_per_cell(self):
        """同一地点误差固定：重复检测给出完全相同的读数，同格内不随机。"""
        stub = OfflineStub(sources=[Source(channel=1, x=800.0, y=0.0, radius=1500.0)],
                           location_error_deg=0.9, error_cell_m=50.0)
        stub.phase = 'running'
        first = decode(stub('/measure', measure_body(1, 0.0, 0.0), 5)[1])
        second = decode(stub('/measure', measure_body(1, 0.0, 0.0), 5)[1])
        self.assertTrue(first['accepted'])
        self.assertEqual(first['measure_result'], 'direction')
        self.assertNotEqual(first['svd_deg'], 0.0)
        self.assertEqual(first['svd_deg'], second['svd_deg'])

    def test_stub_rejects_wrong_robot_and_bad_channel(self):
        """桩必须拒绝身份不符与非法频道，避免把协议错误当成算法结果。"""
        stub = OfflineStub(sources=[Source(channel=1, x=100.0, y=0.0)])
        stub.phase = 'running'
        self.assertFalse(decode(stub('/measure', measure_body(1, 0.0, 0.0, robot_id='other'), 5)[1])['accepted'])
        self.assertEqual(stub('/measure', measure_body(0, 0.0, 0.0), 5)[0], 400)
        self.assertEqual(stub('/measure', measure_body(21, 0.0, 0.0), 5)[0], 400)


def measure_body(channel, x, y, robot_id='offline-team'):
    """构造一条符合桩校验要求的 measure 请求体（直接驱动桩时使用）。"""
    import uuid
    return encode({'arena_id': 'default', 'robot_id': robot_id,
                   'request_id': uuid.uuid4().hex,
                   'position': {'x': x, 'y': y}, 'channel': channel})


class StrategyGuardTests(unittest.TestCase):
    """决策五的一致性护栏与清除候选筛选。"""

    def setUp(self):
        """最小上下文：策略只读取位置、频道与虚拟时间。"""
        self.strategy = Problem3Strategy(_DummyContext())

    def test_finished_is_none_before_any_observation(self):
        """初始状态下既没清满也没全部解决，不判完成。"""
        self.assertIsNone(self.strategy._finished())

    def test_finished_flags_inconsistent_channel(self):
        """矛盾频道必须单独报出，且记录里可追溯。"""
        probe = self.strategy.channels[7]
        probe.observe_direction(0.0, 0.0, 0.0)
        probe.mask[:] = False
        self.assertEqual(self.strategy._finished(), 'model_inconsistent')
        self.assertEqual(self.strategy._inconsistent_channels(), [7])
        self.assertEqual(self.strategy._result()['inconsistent_channels'], [7])

    def test_clear_candidates_include_near_only_channel(self):
        """只拿到距离过近读数的频道必须进入清除候选（回归：曾整个被跳过）。"""
        probe = self.strategy.channels[5]
        probe.observe_near(0.0, 0.0)
        self.assertEqual(self.strategy._clear_candidates((0.0, 0.0)), [5])

    def test_clear_candidates_ignore_unobserved_channel(self):
        """没有任何读数的频道不进入清除候选。"""
        self.assertEqual(self.strategy._clear_candidates((0.0, 0.0)), [])

    def test_observed_channels_covers_near_and_no_signal(self):
        """清除候选来源按"有过任意读数"筛选，而不是只看示向度。"""
        self.strategy.channels[3].observe_no_signal(1500.0, 0.0)
        self.strategy.channels[4].observe_near(0.0, 0.0)
        self.assertEqual(self.strategy._observed_channels(), [3, 4])


class NewModelStrategyTests(unittest.TestCase):
    """新建模落地：保证性追踪边界及可选公平调度。"""

    def setUp(self):
        """构造不连接模拟器的最小问题三策略。"""
        self.strategy = Problem3Strategy(_DummyContext())

    def test_guaranteed_tracking_contracts_upper_bound(self):
        """保证性追踪点必须按 qU 前进，并把下一轮安全上界收缩为 qU。"""
        self.strategy._apply_measure(
            1, np.array([0.0, 0.0]),
            {'measure_result': 'direction', 'svd_deg': 0.0, 'virtual_time_s': 5.0})
        plan = self.strategy._guaranteed_tracking_plan(
            1, self.strategy.channels[1])
        ratio = self.strategy._tracking_ratio()
        self.assertEqual(plan.kind, 'guaranteed_track')
        self.assertAlmostEqual(plan.waypoint[0], 1500.0 * ratio, places=6)
        self.assertAlmostEqual(plan.waypoint[1], 0.0, places=6)
        self.assertAlmostEqual(plan.guaranteed_upper_m, 1500.0 * ratio, places=6)

    def test_tracking_upper_bound_can_certify_clear(self):
        """追踪上界降到20米内时，当前锚点必须进入可靠清除候选。"""
        self.strategy._apply_measure(
            1, np.array([20.0, 10.0]),
            {'measure_result': 'direction', 'svd_deg': 45.0, 'virtual_time_s': 5.0},
            guaranteed_upper_m=15.0)
        self.assertIn(1, self.strategy._clear_candidates((20.0, 10.0)))

    def test_due_tracking_overrides_lower_scored_search(self):
        """等待到期的保证性追踪必须越过一般评分，防止优化动作无限拖延。"""
        self.strategy._apply_measure(
            1, np.array([0.0, 0.0]),
            {'measure_result': 'direction', 'svd_deg': 0.0, 'virtual_time_s': 5.0})
        due = self.strategy._guaranteed_tracking_plan(1, self.strategy.channels[1])
        cheap_search = StopPlan('sweep', np.array([0.0, 0.0]), [2], score_s=0.1)
        self.strategy.tracking_wait_rounds[1] = self.strategy.config.tracking_debt_limit_rounds
        self.strategy._chase_options = lambda: [due]
        self.strategy._search_plan = lambda: cheap_search
        self.assertEqual(self.strategy._decide().kind, 'guaranteed_track')

    def test_default_mode_does_not_force_tracking_debt(self):
        """性能模式默认不让保守追踪债务打断更便宜的搜索动作。"""
        self.strategy._apply_measure(
            1, np.array([0.0, 0.0]),
            {'measure_result': 'direction', 'svd_deg': 0.0, 'virtual_time_s': 5.0})
        self.strategy.tracking_wait_rounds[1] = self.strategy.config.tracking_debt_limit_rounds
        cheap_search = StopPlan('sweep', np.array([0.0, 0.0]), [2], score_s=0.1)
        self.strategy._chase_options = lambda: []
        self.strategy._search_plan = lambda: cheap_search
        self.assertEqual(self.strategy._decide().kind, 'sweep')


class CoverageCertificateTests(unittest.TestCase):
    """不存在性证明的覆盖证书：必须能区分"真覆盖"与"只是掩码空了"。"""

    def setUp(self):
        self.strategy = Problem3Strategy(_DummyContext())

    def test_candidate_grid_is_coverage_complete(self):
        """默认九点格网候选必须对任意源位置都能给出读数。

        最坏距离必须不超过有效接收半径下界 1000 m，这是"候选点全无信号 ⇒
        该频道不存在"这条严格结论的依据。
        """
        complete, worst, note = self.strategy.verify_coverage_completeness()
        self.assertTrue(complete, f'候选格网不具备覆盖完备性：{note}')
        self.assertLessEqual(worst, MIN_RECEIVE_RADIUS_M)
        self.assertGreater(worst, 700.0, '最坏距离过小，采样可能失真')

    def test_candidates_exclude_out_of_region_points(self):
        """候选集必须剔除落在目标圆域外的格点（曾混入 4 个半径 2400 的点）。

        默认 hybrid15 = 九点方格 ∪ 七点六边形去重，共 15 个候选。
        """
        radii = np.hypot(self.strategy.candidates[:, 0], self.strategy.candidates[:, 1])
        self.assertLessEqual(radii.max(), TARGET_RADIUS_M + 1e-9)
        self.assertEqual(len(self.strategy.candidates), 15)

    def test_certificate_rejects_insufficient_coverage(self):
        """只测一个点就判"不存在"时，证书必须拒绝认证。"""
        probe = self.strategy.channels[6]
        probe.observe_no_signal(0.0, 0.0)
        certificate = self.strategy._exclusion_certificate(6)
        self.assertFalse(certificate['certified'])
        self.assertEqual(certificate['no_signal_stations'], 1)

    def test_certificate_accepts_full_coverage(self):
        """候选点全测后掩码为空，证书必须给出认证，且覆盖半径不超过 1000 m。"""
        probe = self.strategy.channels[6]
        for x, y in self.strategy.candidates:
            probe.observe_no_signal(x, y)
        self.assertTrue(probe.is_excluded)
        certificate = self.strategy._exclusion_certificate(6)
        self.assertTrue(certificate['certified'])
        self.assertEqual(certificate['no_signal_stations'], len(self.strategy.candidates))
        self.assertLessEqual(certificate['covering_radius_m'], MIN_RECEIVE_RADIUS_M)

    def test_certificates_skip_cleared_channels(self):
        """已清除频道不属于"不存在"结论，不得出现在覆盖证书里。"""
        self.strategy.channels[6].mark_cleared(0.0)
        self.assertNotIn(6, self.strategy._exclusion_certificates())

    def test_finished_requires_certified_exclusions(self):
        """掩码全空但无覆盖证据时不得收工，避免假阳性的 all_channels_resolved。"""
        # 所有频道都只拿到一条中心处的无信号读数：掩码非空 → 本来就不可收工。
        # 这里把掩码强行清空来模拟"覆盖不足却掩码空了"的病态状态。
        for channel in range(1, CHANNEL_COUNT + 1):
            probe = self.strategy.channels[channel]
            probe.observe_no_signal(0.0, 0.0)
            probe.mask[:] = False
            probe.plan_mask[:] = False
            probe.status = 'unknown'
        certificate = self.strategy._exclusion_certificate(6)
        self.assertFalse(certificate['certified'])
        self.assertNotEqual(self.strategy._finished(), 'all_channels_resolved')

    def test_finished_accepts_certified_exclusions(self):
        """全部频道要么已清除、要么有覆盖证据时，才允许收工。"""
        for channel in range(1, CHANNEL_COUNT + 1):
            probe = self.strategy.channels[channel]
            for x, y in self.strategy.candidates:
                probe.observe_no_signal(x, y)
        self.assertEqual(self.strategy._finished(), 'all_channels_resolved')


class FollowUpEstimateTests(unittest.TestCase):
    """后续清除代价的口径：必须随停靠点变化，不能是全域常数。"""

    def setUp(self):
        self.strategy = Problem3Strategy(_DummyContext())

    def test_follow_up_varies_with_waypoint(self):
        """不同停靠点的后续代价必须不同（回归：曾对任何点都返回同一常数）。"""
        near = self.strategy._follow_up_seconds((0.0, 0.0))
        far = self.strategy._follow_up_seconds((1500.0, 0.0))
        self.assertGreater(near, 0.0)
        self.assertNotAlmostEqual(near, far, places=3)

    def test_follow_up_uses_reachable_region_only(self):
        """后续代价只对该停靠点能覆盖到的区域取平均，量级应在数百米以内。"""
        near = self.strategy._follow_up_seconds((0.0, 0.0))
        # 中心点的接收圆半径 1000 m，平均距离约 2/3*1000 ≈ 667 m ⇒ 约 133 s。
        self.assertLess(near, MIN_RECEIVE_RADIUS_M / ROBOT_SPEED_MPS)
        self.assertGreater(near, 50.0)


class SearchStopFillTests(unittest.TestCase):
    """搜索停靠点补测：移动一旦确定，应利用已经支付的行程成本。"""

    def setUp(self):
        self.strategy = Problem3Strategy(_DummyContext())

    def test_selected_stop_includes_positive_gain_below_threshold(self):
        """低于选点阈值但仍有正增益的频道应在同一停靠点顺手完成。"""
        waypoint = np.array([0.0, 0.0])
        self.strategy._candidate_waypoints = lambda: np.asarray([waypoint])
        self.strategy._active_channels = lambda: [1, 2]
        self.strategy._station_is_new = lambda channel, point: True
        self.strategy._search_options = (
            lambda point, min_gain, cap: [1] if min_gain > 0.0 else [1, 2])
        self.strategy._search_cap = lambda: CHANNEL_COUNT
        self.strategy._measure_gain = lambda channels, point: float(len(channels)) * 1.0e6
        self.strategy._expected_finds = lambda remaining, gain: 1.0
        self.strategy._follow_up_seconds = lambda point: 0.0
        plan = self.strategy._best_search_plan(remaining_sources=5, min_gain=1.0e5)
        self.assertIsNotNone(plan)
        self.assertEqual(plan.measures, [1, 2])


class RecordPlacementTests(unittest.TestCase):
    """问题三离线记录必须与官方在线结果严格分目录保存。"""

    def test_offline_solve_writes_record_under_protocol(self):
        """本地桩运行应写入 protocol，并使用 offline- 文件名前缀。"""
        record = {
            'cleared_channels': [], 'start_virtual_time_s': 0.0,
            'end_virtual_time_s': 0.0, 'real_elapsed_s': 0.0,
            'stop_reason': 'all_channels_resolved', 'unresolved_channels': [],
            'inconsistent_channels': [], 'steps': [], 'moved_distance_m': 0.0,
            'planner_errors': 0,
        }
        context = StrategyContext(RobotClient('offline-team', OfflineStub()), 3)
        self.assertTrue(is_offline_run(context))
        with tempfile.TemporaryDirectory() as tmp:
            with (mock.patch('codes.problem3_solution.run_mission', return_value=record),
                  mock.patch.object(settings, 'PROTOCOL_LOG_DIR', Path(tmp))):
                summary = solve(context)
            target = Path(summary['record_path'])
            self.assertEqual(target.parent, Path(tmp))
            self.assertTrue(target.name.startswith('offline-mission_p3_'))
            payload = json.loads(target.read_text(encoding='utf-8'))
            self.assertEqual(payload['record']['stop_reason'], 'all_channels_resolved')


class BackstopSearchTests(unittest.TestCase):
    """覆盖兜底：只在确有连通空洞时触发，且不得为整片残余横穿全场。"""

    def setUp(self):
        self.strategy = Problem3Strategy(_DummyContext())

    def test_no_backstop_when_nothing_pending(self):
        """所有频道都已清除或已排除时，兜底不得给出任何动作。"""
        for channel in range(1, CHANNEL_COUNT + 1):
            probe = self.strategy.channels[channel]
            for x, y in self.strategy.candidates:
                probe.observe_no_signal(x, y)
        self.assertTrue(all(not knowledge.is_active
                            for knowledge in self.strategy.channels.values()))
        self.assertIsNone(self.strategy._backstop_search_plan(5))

    def test_no_backstop_below_area_threshold(self):
        """残余空洞小于阈值时兜底不触发，避免为零星毛刺专门跑一趟。"""
        self.strategy.channels[6].observe_no_signal(0.0, 0.0)
        for channel in range(1, CHANNEL_COUNT + 1):
            if channel != 6:
                self.strategy.channels[channel].mark_cleared(0.0)
        # 抬高阈值使残余不达标。
        tight = Problem3Strategy(_DummyContext(),
                                Problem3Config(backstop_min_hole_m2=1.0e12))
        tight.channels[6].observe_no_signal(0.0, 0.0)
        for channel in range(1, CHANNEL_COUNT + 1):
            if channel != 6:
                tight.channels[channel].mark_cleared(0.0)
        self.assertIsNone(tight._backstop_search_plan(5))

    def test_dynamic_endgame_competes_with_fixed_grid(self):
        """达到源数下界后，动态空洞方案应能替代更慢的固定格点方案。"""
        self.strategy.config = Problem3Config(route_endgame_min_clears=14)
        for channel in range(1, MIN_SOURCE_COUNT + 1):
            self.strategy.channels[channel].mark_cleared(0.0)
            self.strategy.cleared.add(channel)
        fixed = StopPlan('sweep', np.array([1200.0, 1200.0]), [11], score_s=500.0)
        dynamic = StopPlan('backstop', np.array([300.0, 200.0]), [11], score_s=120.0)
        with (mock.patch.object(self.strategy, '_best_search_plan', return_value=fixed),
              mock.patch.object(self.strategy, '_backstop_search_plan', return_value=dynamic)):
            plan = self.strategy._search_plan()
        self.assertIs(plan, dynamic)

    def test_dynamic_endgame_waits_until_source_lower_bound(self):
        """清除不足十个源时仍保持常规搜索，避免过早转入不存在性收尾。"""
        self.strategy.config = Problem3Config(route_endgame_min_clears=14,
                                              dynamic_endgame_min_clears=10)
        for channel in range(1, MIN_SOURCE_COUNT):
            self.strategy.channels[channel].mark_cleared(0.0)
            self.strategy.cleared.add(channel)
        fixed = StopPlan('sweep', np.array([1200.0, 0.0]), [10], score_s=200.0)
        with (mock.patch.object(self.strategy, '_best_search_plan', return_value=fixed),
              mock.patch.object(self.strategy, '_backstop_search_plan') as dynamic_mock):
            plan = self.strategy._search_plan()
        self.assertIs(plan, fixed)
        dynamic_mock.assert_not_called()

    def test_farthest_point_candidates_cover_both_ends(self):
        """最远点采样应覆盖狭长空洞两端，而不是集中在数组局部。"""
        points = np.column_stack((np.arange(11, dtype=float), np.zeros(11)))
        candidates = self.strategy._farthest_point_candidates(points, 3)
        self.assertEqual(len(candidates), 3)
        self.assertAlmostEqual(float(candidates[:, 0].min()), 0.0)
        self.assertAlmostEqual(float(candidates[:, 0].max()), 10.0)

    def test_route_endgame_selects_lowest_full_route_cost(self):
        """路线兜底应按完整补证代价选首站，而不是沿用单站信息增益排序。"""
        candidates = np.array([[-1200.0, 0.0], [1200.0, -1200.0]])
        self.strategy._candidate_waypoints = lambda: candidates
        self.strategy._search_options = lambda waypoint, min_gain, cap: [1]
        self.strategy._search_cap = lambda: CHANNEL_COUNT
        self.strategy._absence_route_seconds = (
            lambda waypoint, measures, active: 100.0 if waypoint[0] > 0.0 else 500.0)
        plan = self.strategy._route_aware_endgame_plan()
        self.assertIsNotNone(plan)
        self.assertEqual(plan.kind, 'route_backstop')
        np.testing.assert_allclose(plan.waypoint, [1200.0, -1200.0])

    def test_absence_route_simulation_does_not_mutate_real_masks(self):
        """无信号情景前瞻只能修改掩码副本，不得伪造观测或覆盖证书。"""
        original = self.strategy.channels[1].mask.copy()
        observations = len(self.strategy.channels[1].observations)
        score = self.strategy._absence_route_seconds(
            np.array([0.0, 0.0]), [1], [1])
        self.assertTrue(np.isfinite(score))
        np.testing.assert_array_equal(self.strategy.channels[1].mask, original)
        self.assertEqual(len(self.strategy.channels[1].observations), observations)

    def test_detected_channel_disables_route_endgame(self):
        """只要仍有已发现源，路线补证不得抢占正常追踪动作。"""
        for channel in range(1, 15):
            self.strategy.channels[channel].mark_cleared(0.0)
            self.strategy.cleared.add(channel)
        fixed = StopPlan('sweep', np.array([1200.0, 0.0]), [16], score_s=100.0)
        with (mock.patch.object(self.strategy, '_best_search_plan', return_value=fixed),
              mock.patch.object(self.strategy, '_detected_channels', return_value=[15]),
              mock.patch.object(self.strategy, '_route_aware_endgame_plan') as route_mock):
            plan = self.strategy._search_plan()
        self.assertIs(plan, fixed)
        route_mock.assert_not_called()

    def test_backstop_targets_largest_connected_hole(self):
        """存在大块残余空洞时，兜底选点必须朝空洞走，而不是停在已扫过的中心。"""
        # 只在中心做一次无信号：残余是半径 1800 圆挖掉半径 1000 圆的大环带。
        self.strategy.channels[6].observe_no_signal(0.0, 0.0)
        for channel in range(1, CHANNEL_COUNT + 1):
            if channel != 6:
                self.strategy.channels[channel].mark_cleared(0.0)
        plan = self.strategy._backstop_search_plan(5)
        self.assertIsNotNone(plan)
        self.assertEqual(plan.kind, 'backstop')
        # 选点必须朝外走向环带（残余区域在半径 1000 m 以外），而不是停在原点。
        self.assertGreater(np.hypot(plan.waypoint[0], plan.waypoint[1]), 900.0)

    def test_connected_clusters_separates_distant_groups(self):
        """连通聚类必须把相距很远的点分到不同簇。"""
        points = np.array([[0.0, 0.0], [10.0, 0.0], [5000.0, 5000.0]])
        clusters = Problem3Strategy._connected_clusters(points, 100.0)
        self.assertEqual(len(clusters), 2)
        self.assertEqual(sorted(len(cluster) for cluster in clusters), [1, 2])

    def test_connected_clusters_empty_input(self):
        """空输入返回空列表，不得抛异常。"""
        self.assertEqual(Problem3Strategy._connected_clusters(np.empty((0, 2)), 100.0), [])


class _DummyContext:
    """满足策略构造与预算查询所需最小接口的测试替身。"""

    class _State:
        position = (0.0, 0.0)
        channel = 1
        virtual_time_s = 0.0

    def __init__(self):
        self.state = self._State()

    @property
    def remaining_real_duration_s(self):
        return None

    def should_stop(self):
        return False


class OfflineEndToEndTests(unittest.TestCase):
    """离线端到端：固定案例必须全部清除；轮次与停滞保护必须生效。"""

    @classmethod
    def setUpClass(cls):
        """只跑一次固定案例，供同类多个断言复用，控制测试总时长。"""
        cls.scenario = fixed_scenario([(1, 900.0, 0.0), (2, -700.0, 600.0),
                                       (3, 300.0, -1000.0)])
        stub = OfflineStub(sources=cls.scenario.sources)
        client = RobotClient('offline-team', stub,
                             log_path='output/protocol/test-problem3-offline.jsonl')
        cls.result = run_strategy(client, lambda context: run_mission(context), problem=3)
        cls.protocol_cleared = sorted(client.state.cleared_channels)
        cls.stub = stub

    def test_fixed_scenario_clears_every_source(self):
        """三个不同方位的干扰源都应被清除，且无规划异常。"""
        record = self.result['algorithm_result']
        self.assertEqual(sorted(record['cleared_channels']), [1, 2, 3])
        self.assertEqual(self.protocol_cleared, [1, 2, 3])
        self.assertEqual(record['planner_errors'], 0)
        self.assertGreater(record['actions'], 0)
        self.assertIn(record['stop_reason'], ('cleared_limit', 'all_channels_resolved'))

    def test_summary_reports_table1_quantities(self):
        """汇总给出表 1 所需的清除个数、平均定位清除时间与程序运行时间。"""
        summary = summarize(self.result['algorithm_result'],
                            true_total=self.scenario.true_total)
        self.assertEqual(summary['cleared_count'], 3)
        self.assertAlmostEqual(summary['cleared_ratio'], 1.0, places=9)
        self.assertGreater(summary['total_virtual_time_s'], 0.0)
        self.assertAlmostEqual(summary['average_clear_time_s'],
                               summary['total_virtual_time_s'] / 3, places=3)
        self.assertGreaterEqual(summary['program_runtime_s'], 0.0)
        self.assertGreater(summary['moved_distance_m'], 0.0)
        self.assertEqual(summary['inconsistent_channels'], [])
        self.assertIn(summary['stop_reason'], ('cleared_limit', 'all_channels_resolved'))

    def test_source_within_five_metres_of_start_is_cleared(self):
        """起点 5 米内的源首个读数只能是"距离过近"，必须就地清除（回归 H1）。"""
        scenario = fixed_scenario([(5, 3.0, 0.0), (9, 900.0, 0.0)])
        config = Problem3Config(max_rounds=3)
        stub = OfflineStub(sources=scenario.sources)
        client = RobotClient('offline-team', stub)
        result = run_strategy(client, lambda context: run_mission(context, config), problem=3)
        record = result['algorithm_result']
        self.assertIn(5, record['cleared_channels'])
        self.assertEqual(record['planner_errors'], 0)
        self.assertEqual(record['inconsistent_channels'], [])
        results = [step['result'] for step in record['steps'] if step['channel'] == 5]
        self.assertIn('near', results)

    def test_record_is_json_serializable_and_complete(self):
        """任务记录必须可序列化，且包含轨迹、频道序列与配置快照。"""
        import json
        record = self.result['algorithm_result']
        text = json.dumps(record, ensure_ascii=False)
        self.assertEqual(len(json.loads(text)['steps']), record['actions'])
        self.assertGreaterEqual(len(record['trajectory']), 1)
        self.assertEqual(sorted(record['channel_series']), list(range(1, 21)))
        self.assertEqual(record['config']['lattice_spacing_m'], 10.0)

    def test_trajectory_matches_protocol_position(self):
        """记录的终点位置必须与协议层位置一致（只有实际执行指令才移动）。"""
        record = self.result['algorithm_result']
        last = record['trajectory'][-1]
        self.assertAlmostEqual(float(last[0]), float(self.stub.position[0]), places=6)
        self.assertAlmostEqual(float(last[1]), float(self.stub.position[1]), places=6)

    def test_round_limits_terminate_safely(self):
        """限制轮次与停滞上限后必须安全退出，不得超发动作或抛异常。"""
        scenario = fixed_scenario([(1, 1400.0, 0.0)])
        config = Problem3Config(lattice_spacing_m=20.0, planning_spacing_m=100.0,
                                max_rounds=2, stall_limit=1)
        stub = OfflineStub(sources=scenario.sources)
        client = RobotClient('offline-team', stub)
        result = run_strategy(client, lambda context: run_mission(context, config), problem=3)
        record = result['algorithm_result']
        self.assertLessEqual(record['actions'], 60)
        self.assertIn(record['stop_reason'],
                      ('cleared_limit', 'all_channels_resolved', 'stalled',
                       'no_action', 'exit_margin'))

    def test_no_source_scenario_does_not_hang(self):
        """空案例必须在有限动作内结束，且不会误报已清除。"""
        scenario = fixed_scenario([])
        config = Problem3Config(lattice_spacing_m=20.0, planning_spacing_m=100.0,
                                max_rounds=6, stall_limit=2)
        stub = OfflineStub(sources=scenario.sources)
        client = RobotClient('offline-team', stub)
        result = run_strategy(client, lambda context: run_mission(context, config), problem=3)
        record = result['algorithm_result']
        self.assertEqual(record['cleared_channels'], [])
        self.assertLessEqual(record['actions'], 200)
        self.assertIn(record['stop_reason'],
                      ('all_channels_resolved', 'stalled', 'no_action', 'exit_margin'))


if __name__ == '__main__':
    unittest.main()
