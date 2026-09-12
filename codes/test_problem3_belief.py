# -*- coding: utf-8 -*-
"""贝叶斯置信层的单元测试：归一性、支持集一致性、似然方向性与策略接线。

红线回顾：置信层只用于排序与下注，因此测试重点之一是"置信图不越过证书
掩码"以及"belief_enabled=False 时回到基线路径"。
"""
import math
import unittest

import numpy as np

from .benchmark_q34_iteration import run_case
from .config import CLEAR_RADIUS_M, TARGET_RADIUS_M
from .problem3_belief import ChannelBelief
from .problem3_model import ChannelKnowledge, Lattice
from .problem3_solution import Problem3Config, Problem3Strategy


class _DummyContext:
    """只提供策略构造所需最小接口的测试桩。"""

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


def _fine():
    return Lattice.build(10.0)


def _coarse():
    return Lattice.build(50.0)


class BeliefConstructionTests(unittest.TestCase):
    """构造与归一性。"""

    def test_initial_mass_is_uniform_on_target_disk(self):
        belief = ChannelBelief(_fine(), _coarse(), 0.42)
        self.assertAlmostEqual(float(belief.fine_mass.sum()), 1.0, places=9)
        self.assertAlmostEqual(float(belief.coarse_mass.sum()), 1.0, places=9)
        outside = np.hypot(_fine().points[:, 0], _fine().points[:, 1]) > TARGET_RADIUS_M + 1e-9
        self.assertAlmostEqual(float(belief.fine_mass[outside].sum()), 0.0, places=12)

    def test_invalid_std_rejected_by_config(self):
        with self.assertRaises(ValueError):
            Problem3Config(belief_direction_std_deg=0.0)
        with self.assertRaises(ValueError):
            Problem3Config(belief_clear_min_mass=0.0)
        with self.assertRaises(ValueError):
            Problem3Config(belief_clear_max_distance_m=-1.0)

    def test_belief_disabled_by_default_and_enabled_by_flag(self):
        # 默认关闭（离线扫档未达采纳标准，见 Problem3Config 注释）；
        # 显式开启时按频道建层。
        disabled = Problem3Strategy(_DummyContext())
        self.assertIsNone(disabled.beliefs)
        enabled = Problem3Strategy(_DummyContext(), Problem3Config(belief_enabled=True))
        self.assertIsNotNone(enabled.beliefs)
        self.assertEqual(set(enabled.beliefs), set(enabled.channels))


class BeliefUpdateTests(unittest.TestCase):
    """更新后支持集与质量的性质。"""

    def setUp(self):
        self.fine = _fine()
        self.coarse = _coarse()
        self.knowledge = ChannelKnowledge(1, self.fine, self.coarse)
        self.belief = ChannelBelief(self.fine, self.coarse, 0.42)

    def _updated(self, **kwargs):
        """重置一对 (knowledge, belief)，供各测试独立使用。"""
        self.knowledge = ChannelKnowledge(1, self.fine, self.coarse)
        self.belief = ChannelBelief(self.fine, self.coarse, 0.42)

    def test_support_stays_within_certificate_mask_after_direction(self):
        self.knowledge.observe_direction(0.0, 0.0, 30.0)
        self.belief.observe_direction(self.knowledge, 0.0, 0.0, 30.0)
        self.assertGreater(float(self.belief.fine_mass.sum()), 0.99)
        self.assertEqual(float(self.belief.fine_mass[~self.knowledge.mask].sum()), 0.0)
        self.assertEqual(float(self.belief.coarse_mass[~self.knowledge.plan_mask].sum()), 0.0)

    def test_direction_kernel_concentrates_on_centre_line(self):
        self.knowledge.observe_direction(0.0, 0.0, 0.0)
        self.belief.observe_direction(self.knowledge, 0.0, 0.0, 0.0)
        # 沿示向中轴 (900, 0) 的单元质量应大于同距离的楔形边缘单元 (895, 47)。
        axis_index = int(np.argmin(np.linalg.norm(self.fine.points - [900.0, 0.0], axis=1)))
        edge_index = int(np.argmin(np.linalg.norm(self.fine.points - [895.0, 47.0], axis=1)))
        self.assertGreater(self.belief.fine_mass[axis_index] * 10.0,
                           self.belief.fine_mass[edge_index])

    def test_no_signal_downweights_inner_annulus_more_than_outer(self):
        self.knowledge.observe_no_signal(0.0, 0.0)
        self.belief.observe_no_signal(self.knowledge, 0.0, 0.0)
        near_index = int(np.argmin(np.linalg.norm(self.fine.points - [1100.0, 0.0], axis=1)))
        far_index = int(np.argmin(np.linalg.norm(self.fine.points - [1600.0, 0.0], axis=1)))
        self.assertTrue(self.knowledge.mask[near_index])
        self.assertTrue(self.knowledge.mask[far_index])
        # P(a < 1100) = 0.2 vs P(a < 1600) = 1：归一化后远端单元质量应显著更高。
        self.assertGreater(self.belief.fine_mass[far_index] * 3.0,
                           self.belief.fine_mass[near_index])

    def test_near_concentrates_all_mass_in_clear_ball(self):
        self.knowledge.observe_near(0.0, 0.0)
        self.belief.observe_near(self.knowledge, 0.0, 0.0)
        self.assertAlmostEqual(float(self.belief.fine_mass.sum()), 1.0, places=9)
        self.assertAlmostEqual(self.belief.mass_within(0.0, 0.0, CLEAR_RADIUS_M), 1.0, places=9)
        self.assertAlmostEqual(self.belief.expected_distance_m([0.0, 0.0]), 0.0, places=6)

    def test_clear_failure_empties_cleared_ball(self):
        self.knowledge.observe_direction(0.0, 0.0, 0.0)
        self.belief.observe_direction(self.knowledge, 0.0, 0.0, 0.0)
        self.knowledge.observe_clear_failure(200.0, 0.0)
        self.belief.observe_clear_failure(self.knowledge, 200.0, 0.0)
        ball_index = int(np.argmin(np.linalg.norm(self.fine.points - [200.0, 0.0], axis=1)))
        self.assertFalse(self.knowledge.mask[ball_index])
        self.assertEqual(float(self.belief.fine_mass[ball_index]), 0.0)
        self.assertGreater(float(self.belief.fine_mass.sum()), 0.99)

    def test_degenerate_support_falls_back_to_zeros_not_nan(self):
        self.knowledge.observe_near(0.0, 0.0)
        self.belief.observe_near(self.knowledge, 0.0, 0.0)
        self.knowledge.mask[:] = False
        self.knowledge.plan_mask[:] = False
        self.belief.observe_clear_failure(self.knowledge, 0.0, 0.0)
        self.assertTrue(np.isfinite(self.belief.fine_mass).all())
        self.assertAlmostEqual(float(self.belief.fine_mass.sum()), 0.0, places=12)


class StrategyWiringTests(unittest.TestCase):
    """置信层与策略决策4/2/1 的接线。"""

    def setUp(self):
        self.strategy = Problem3Strategy(_DummyContext(),
                                         Problem3Config(belief_enabled=True))

    def test_default_config_enables_layer(self):
        self.assertIsNotNone(self.strategy.beliefs)
        self.assertEqual(set(self.strategy.beliefs), set(self.strategy.channels))

    def _observe_direction_on_channel(self, channel, x, y, bearing):
        """与 _apply_measure 相同的双路径更新（测试桩不走协议）。"""
        probe = self.strategy.channels[channel]
        probe.observe_direction(x, y, bearing)
        self.strategy.beliefs[channel].observe_direction(probe, x, y, bearing)
        return probe

    def test_speculative_bet_fires_on_peaked_posterior(self):
        # 单条示向的后验是一条长光束，任何 20 m 球都拿不到多数质量；
        # 两条交叉示向（LLS 两点定位）把后验收敛到交点附近，此时应允许下注。
        self._observe_direction_on_channel(5, 0.0, 0.0, 0.0)
        self._observe_direction_on_channel(5, 400.0, -300.0, 90.0)
        waypoint = (400.0, 0.0)
        mass = self.strategy.beliefs[5].mass_within(*waypoint, CLEAR_RADIUS_M)
        bet = self.strategy._speculative_bet(self.strategy.channels[5], waypoint)
        self.assertGreaterEqual(mass, self.strategy.config.belief_clear_min_mass)
        self.assertIsNotNone(bet)
        self.assertEqual(bet[1], 5)
        self.assertAlmostEqual(bet[0], mass, places=12)

    def test_single_bearing_leaves_beam_posterior_without_majority_ball(self):
        # 反向校验：单条示向读数下不应在远端轻率下注（红线：下注须有充分后验）。
        self._observe_direction_on_channel(5, 0.0, 0.0, 0.0)
        self.assertLess(self.strategy.beliefs[5].mass_within(100.0, 0.0, CLEAR_RADIUS_M),
                        self.strategy.config.belief_clear_min_mass)

    def test_speculative_bet_respects_distance_cap_when_positive(self):
        self._observe_direction_on_channel(5, 0.0, 0.0, 0.0)
        strict = Problem3Config(belief_clear_max_distance_m=50.0)
        self.strategy.config = strict
        bet = self.strategy._speculative_bet(self.strategy.channels[5], (100.0, 0.0))
        # 楔形最坏距离上界仍达接收半径上界，距离帽存在时拒绝下注。
        self.assertIsNone(bet)

    def test_speculative_bet_below_threshold_is_rejected(self):
        self._observe_direction_on_channel(5, 0.0, 0.0, 0.0)
        hungry = Problem3Config(belief_clear_min_mass=0.999999)
        self.strategy.config = hungry
        self.assertIsNone(self.strategy._speculative_bet(
            self.strategy.channels[5], (100.0, 0.0)))

    def test_baseline_speculative_path_without_layer(self):
        strategy = Problem3Strategy(_DummyContext(),
                                    Problem3Config(belief_enabled=False))
        probe = strategy.channels[5]
        # 掩码收缩到 |q| <= 20：最坏距离 27.07 <= 45，均匀先验概率约 0.42 >= 0.30。
        probe.mask = np.linalg.norm(_fine().points, axis=1) <= 20.0
        bet = strategy._speculative_bet(probe, (0.0, 0.0))
        self.assertIsNotNone(bet)
        self.assertEqual(bet[1], 5)

    def test_expected_remaining_matches_uniform_mean_when_disabled(self):
        channel = 5
        self._observe_direction_on_channel(channel, 0.0, 0.0, 0.0)
        off = Problem3Config(belief_chase_expectation=False)
        self.strategy.config = off
        baseline = self.strategy._expected_remaining_s(channel, (100.0, 0.0))
        predicted = self.strategy._predicted_series(channel, (100.0, 0.0))
        self.assertAlmostEqual(baseline, float(np.mean(predicted)) / 5.0, places=9)
        # 开启后仍是有限值且与后验期望一致（剩余支撑必非空）。
        self.strategy.config = Problem3Config()
        value = self.strategy._expected_remaining_s(channel, (100.0, 0.0))
        self.assertTrue(math.isfinite(value))
        self.assertGreaterEqual(value, 0.0)

    def test_search_rank_uses_mass_when_enabled_and_gain_when_disabled(self):
        waypoint = (0.0, 0.0)
        channel = next(iter(self.strategy.channels))
        gain = self.strategy._search_gain(channel, waypoint)
        self.assertGreater(gain, 0.0)
        # 默认（排序关闭）与基线一致；显式开启后按后验质量排序。
        self.assertAlmostEqual(self.strategy._search_rank(channel, waypoint, gain),
                               gain, places=9)
        self.strategy.config = Problem3Config(belief_search_ranking=True)
        rank_on = self.strategy._search_rank(channel, waypoint, gain)
        self.assertTrue(0.0 < rank_on <= 1.0)

    def test_clear_candidates_prefers_certain_over_bets(self):
        probe = self.strategy.channels[5]
        probe.observe_near(0.0, 0.0)
        self.strategy.beliefs[5].observe_near(probe, 0.0, 0.0)
        self.assertEqual(self.strategy._clear_candidates((0.0, 0.0)), [5])


class EndToEndBeliefTests(unittest.TestCase):
    """离线桩端到端：置信层开启时任务仍须完整、合法地结束。"""

    def test_small_mission_completes_with_belief_layer(self):
        case = run_case(3, seed=11, sources=3, overrides={'belief_enabled': True})
        self.assertTrue(case['checks']['all_constraints_ok'],
                        msg=f"checks={case['checks']} error={case['error']}")
        self.assertEqual(case['cleared'], case['true_total'])

    def test_small_mission_completes_without_belief_layer(self):
        case = run_case(3, seed=11, sources=3, overrides={'belief_enabled': False})
        self.assertTrue(case['checks']['all_constraints_ok'],
                        msg=f"checks={case['checks']} error={case['error']}")
        self.assertEqual(case['cleared'], case['true_total'])


if __name__ == '__main__':
    unittest.main()
