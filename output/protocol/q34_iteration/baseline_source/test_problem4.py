# -*- coding: utf-8 -*-
"""问题四测试：几何证书、联合外包、动作与完成证据、入口与绘图。

普通unittest只用内存上下文或本地桩；较长端到端用例需显式设置RUN_PROBLEM4_E2E=1。
任何用例都不会连接官方模拟器。
"""
import contextlib
import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

from . import config as settings
from .config import (MIN_RECEIVE_RADIUS_M, PROBLEM4_OUTPUT_DIR, PROTOCOL_LOG_DIR,
                     TARGET_RADIUS_M, record_dir_for)
from .offline_stub import OfflineStub, Source
from .problem3_model import Lattice
from .problem4_model import (Problem4Knowledge, optimized_search_waypoints,
                             triangular_search_waypoints)
from .problem4_solution import (Problem4Config, Problem4Strategy, StopPlan,
                                build_scenario, feedback_score, is_offline_run,
                                main, run_mission, solve, summarize)
from .protocol import HttpTransport, RobotClient
from .scenario import fixed_scenario
from .scenario_p4 import boundary_scenario, mixed_scenario
from .strategy import BudgetReached, StrategyContext, run_strategy

try:
    import matplotlib  # noqa: F401
    HAVE_MATPLOTLIB = True
except Exception:  # pragma: no cover - 取决于本机环境
    HAVE_MATPLOTLIB = False


class MemoryContext:
    """不执行任何请求的规划上下文；动作方法必须由特定测试显式提供。"""

    def __init__(self, remaining=1200):
        """保存公开状态和剩余预算。"""
        self.problem = 4
        self.remaining_real_duration_s = remaining
        self.state = SimpleNamespace(position=(0.0, 0.0), channel=1, virtual_time_s=0.0,
                                     max_virtual_duration_s=360000)

    def should_stop(self):
        """与公共上下文一致，预留正常退出时间。"""
        return self.remaining_real_duration_s <= 10

    def measure(self, x, y, channel):
        """默认禁止动作，避免规划单元测试意外执行请求。"""
        raise AssertionError('此测试不允许检测动作。')

    def clear(self, x, y, channel):
        """默认禁止清除。"""
        raise AssertionError('此测试不允许清除动作。')


class JointKnowledgeTests(unittest.TestCase):
    """针对会造成定向漏检和错误可靠判断的边界构造。"""

    def setUp(self):
        """用小型已知位置单元隔离联合约束，避免每例建立整域数组。"""
        points = np.array([[0.0, 0.0], [40.0, 0.0], [0.0, 40.0]])
        self.lattice = Lattice(points, 1.0, 100.0, 0.0)
        self.knowledge = Problem4Knowledge(1, self.lattice)

    def test_direction_is_not_emission_orientation(self):
        """右侧看到180度示向，真源仍可能向东发射，不应把两角混同。"""
        self.knowledge.observe('direction', 100, 0, bearing_deg=180)
        self.knowledge.observe('no_signal', -100, 0)
        points = self.knowledge.possible_points()
        self.assertTrue(np.any(np.all(points == [0, 0], axis=1)))
        self.assertEqual(self.knowledge.status, 'detected')
        self.assertEqual(self.knowledge.type_status, 'directional')

    def test_negative_history_is_replayed(self):
        """先背面无信号、再正面near，重放历史应保留定向真值。"""
        self.knowledge.observe('no_signal', -100, 0)
        self.assertEqual(self.knowledge.status, 'unknown')
        self.knowledge.observe('near', 1, 0)
        self.assertEqual(self.knowledge.type_status, 'directional')
        self.assertTrue(self.knowledge.near_certain_clear(1, 0))

    def test_direction_filters_position_wedge(self):
        """位于错误角域的候选位置必须排除，与类型无关。"""
        self.knowledge.observe('direction', 100, 0, bearing_deg=180)
        self.assertFalse(np.any(np.all(self.knowledge.possible_points() == [0, 40], axis=1)))

    def test_clear_failure_removes_fallback_cell_only_after_feedback(self):
        """后备选点不能提前改状态，失败后才删20米内单元。"""
        self.knowledge.observe('direction', 100, 0, bearing_deg=180)
        before = self.knowledge.possible_points()
        point = self.knowledge.fallback_point(np.array([0, 0]))
        np.testing.assert_array_equal(before, self.knowledge.possible_points())
        self.knowledge.observe_clear_failure(*point)
        self.assertFalse(np.any(np.all(self.knowledge.possible_points() == point, axis=1)))
        self.assertTrue(self.knowledge.cleared_here(*point))
        self.assertIsNone(self.knowledge.measured_at(*point))

    def test_unknown_existence_not_promoted_to_reliable_clear(self):
        """即使小型测试位置全在20米内，未发现也不能保证清除成功。"""
        lattice = Lattice(np.array([[0.0, 0.0]]), 1.0, 1.0, 0.0)
        self.assertFalse(Problem4Knowledge(1, lattice).certain_clear(0, 0))

    def test_empty_detected_model_is_inconsistent(self):
        """互相矛盾的近场观测导致异常，不能当成频道不存在。"""
        self.knowledge.observe('near', 0, 0)
        self.knowledge.observe('near', 100, 100)
        self.assertEqual(self.knowledge.status, 'inconsistent')

    def test_covering_radius_is_added_to_clear_bound(self):
        """格点中心落在20米内不够，必须连同单元覆盖半径计入。"""
        lattice = Lattice(np.array([[0.0, 0.0]]), 20.0, 20.0, 14.0)
        knowledge = Problem4Knowledge(1, lattice)
        knowledge.observe('near', 1, 0)
        self.assertFalse(knowledge.certain_clear(10, 0))
        self.assertTrue(knowledge.near_certain_clear(1, 0))


class SearchGeometryTests(unittest.TestCase):
    """三角覆盖保留边界外顶点，不能只检查接收圆的并。"""

    def test_containing_triangles_keep_all_vertices(self):
        """按三角格坐标直接构造包围若干圆内/边界点的三角形。"""
        spacing = 900.0
        points = triangular_search_waypoints(spacing)
        self.assertGreater(float(np.linalg.norm(points, axis=1).max()), 1800)
        for radius in (0, 900, 1800):
            for angle in np.linspace(0, 2 * np.pi, 37):
                query = radius * np.array([np.cos(angle), np.sin(angle)])
                v = query[1] / (spacing * np.sqrt(3) / 2)
                u = query[0] / spacing - v / 2
                i, j = int(np.floor(u)), int(np.floor(v))
                indices = ([(i, j), (i + 1, j), (i, j + 1)] if u - i + v - j <= 1
                           else [(i + 1, j + 1), (i + 1, j), (i, j + 1)])
                for a, b in indices:
                    vertex = spacing * np.array([a + b / 2, np.sqrt(3) * b / 2])
                    self.assertLess(float(np.linalg.norm(points - vertex, axis=1).min()), 1e-6)
                    self.assertLessEqual(float(np.linalg.norm(vertex - query)), spacing + 1e-6)

    def test_unsafe_spacing_rejected(self):
        """搜索边长和清除单元尺寸不能破坏理论保证。"""
        with self.assertRaises(ValueError):
            triangular_search_waypoints(1001)
        with self.assertRaises(ValueError):
            Problem4Config(lattice_spacing_m=30)

    def test_ordered_search_route_is_valid_permutation(self):
        """预排路线必须是不重不漏的顶点序列，且不长于最近邻构造。"""
        from .problem4_model import ordered_search_route
        points = triangular_search_waypoints(900.0)
        route = ordered_search_route(points, np.zeros(2))
        self.assertEqual(sorted(route), list(range(len(points))))
        start_distance = float(np.linalg.norm(points[route[0]]))
        self.assertLessEqual(start_distance, 1e-6)

    def test_search_route_config_validation(self):
        """search_route 只允许 greedy/tour，tour 模式预排顺序生效。"""
        with self.assertRaises(ValueError):
            Problem4Config(search_route='spiral')
        config = Problem4Config(search_route='tour')
        strategy = Problem4Strategy.__new__(Problem4Strategy)
        strategy.config = config
        strategy.waypoints = triangular_search_waypoints(config.search_spacing_m)
        from .problem4_model import ordered_search_route
        strategy.search_route_order = ordered_search_route(strategy.waypoints, np.zeros(2))
        self.assertEqual(len(strategy.search_route_order), len(strategy.waypoints))

    def test_no_signal_participates_in_worst_feedback(self):
        """全部样本在背面时，评分应保留全部位置的不确定性。"""
        hypotheses = np.array([[0, 0, 1000, 1, 0], [40, 0, 1000, 1, 0]], dtype=float)
        radius, gain = feedback_score(hypotheses, np.array([-100, 0]))
        self.assertEqual(gain, 0)
        self.assertGreaterEqual(radius, 20)


class StrategyEvidenceTests(unittest.TestCase):
    """纯内存地检查状态提交、频道和完成判据。"""

    def setUp(self):
        """初始化不允许动作的上下文。"""
        self.context = MemoryContext()
        self.strategy = Problem4Strategy(self.context)

    def test_rejected_action_does_not_move_or_cover(self):
        """拒绝反馈不得提交计划位置或覆盖证据。"""
        point = self.strategy.waypoints[0]
        with self.assertRaises(RuntimeError):
            self.strategy._commit_action(point, 1, 'measure', {'accepted': False})
        np.testing.assert_array_equal(self.strategy.position, [0, 0])
        self.assertEqual(len(self.strategy.steps), 0)
        self.assertEqual(self.strategy.coverage[1], set())

    def test_ten_cleared_is_not_completion(self):
        """下界10不是未知总数问题的完成证据。"""
        for channel in range(1, 11):
            self.strategy.channels[channel].mark_cleared()
        self.assertIsNone(self.strategy.finished())

    def test_unscanned_channel_cannot_borrow_coverage(self):
        """一个频道完成所有网格顶点检测，不能推及其他频道。"""
        self.strategy.coverage[1] = set(range(len(self.strategy.waypoints)))
        self.assertIsNone(self.strategy.finished())
        self.assertEqual(self.strategy.channels[1].status, 'excluded')
        self.assertEqual(self.strategy.channels[2].status, 'unknown')

    def test_sixteen_cleared_is_completion(self):
        """上界16全部清除后可以结束。"""
        for channel in range(1, 17):
            self.strategy.channels[channel].mark_cleared()
        self.assertEqual(self.strategy.finished(), 'cleared_limit')

    def test_budget_limit_is_not_completion(self):
        """剩余时间不足不执行动作，返回未完成。"""
        self.context.remaining_real_duration_s = 1
        result = self.strategy.run()
        self.assertFalse(result['completed'])
        self.assertEqual(result['stop_reason'], 'budget_limit')
        self.assertEqual(result['steps'], [])

    def test_virtual_budget_includes_travel(self):
        """发令前要计入移动耗时，而非只比较当前虚拟时间。"""
        self.context.state.max_virtual_duration_s = 10
        with self.assertRaises(BudgetReached):
            self.strategy._check_action_budget(np.array([100, 0]), 'measure', 1)

    def test_fairness_age_must_be_a_positive_integer(self):
        """公平年龄为零或负数时调度规则失去意义，必须在发令前被拒绝。"""
        for bad in (0, -3, True):
            with self.assertRaises(ValueError):
                Problem4Config(fairness_age_rounds=bad)

    def test_finish_detected_switch_must_be_boolean(self):
        """连续处理开关不能接受会被误当作真假的数值。"""
        with self.assertRaises(ValueError):
            Problem4Config(finish_detected_before_search=1)

    def test_detected_target_postpones_search_and_search_resumes(self):
        """目标未清除时连续追踪，清除后仍恢复原覆盖计划。"""
        self.strategy.round = 4
        self.strategy.channels[1].status = 'detected'
        track = StopPlan(np.array([100.0, 0.0]), 'track', channel=1)
        search = StopPlan(np.array([900.0, 0.0]), 'search', search_index=0)
        with (mock.patch.object(self.strategy, '_search_plan', return_value=search),
              mock.patch.object(self.strategy, '_tracking_plan', return_value=track),
              mock.patch.object(self.strategy, '_next_channel', return_value=1)):
            _, selected = self.strategy.decide_direction()
        self.assertIs(selected, track)
        self.strategy.channels[1].mark_cleared()
        with mock.patch.object(self.strategy, '_search_plan', return_value=search):
            _, selected = self.strategy.decide_direction()
        self.assertIs(selected, search)


class LocalProtocolTests(unittest.TestCase):
    """使用现有内存协议桩检查题面边界，不连接网络。"""

    def test_backside_near_is_silent_but_clear_works(self):
        """背面1米不返回near；清除与朝向无关且不切换测向频道。"""
        stub = OfflineStub(sources=(Source(7, 0, 0, direction_deg=0),))
        client = RobotClient('offline-team', stub)
        client.enter()
        response = client.measure(-1, 0, 7)
        self.assertEqual(response['measure_result'], 'no_signal')
        client.measure(-1, 0, 3)
        self.assertEqual(client.clear(-1, 0, 7)['clear_result'], 'success')
        self.assertEqual(client.state.channel, 3)
        client.exit()


class JointModelGuardTests(unittest.TestCase):
    """联合外包围的保守性：未知频道不得被凭空排除，已清除频道不得继续读。"

    与实际问题四相同的20米格距，用于确认整域规模下的判据与字段行为。
    """

    def setUp(self):
        """使用默认格距的整域格网，暴露全格点乘联合数组时的字段行为。"""
        self.lattice = Lattice.build(20.0)
        self.knowledge = Problem4Knowledge(3, self.lattice)

    def test_no_signal_on_unknown_channel_keeps_full_uncertainty(self):
        """未知频道的无信号既不改状态，也不当作圆外排除证据。"""
        self.knowledge.observe('no_signal', -100, 0)
        self.assertEqual(self.knowledge.status, 'unknown')
        self.assertEqual(self.knowledge.type_status, 'unknown')
        self.assertEqual(len(self.knowledge.possible_points()), len(self.lattice.points))

    def test_observe_is_rejected_after_cleared(self):
        """清除成功后不能再登记检测，避免在已释放的联合数组上更新。"""
        self.knowledge.observe('near', 1, 0)
        self.knowledge.mark_cleared()
        with self.assertRaises(ValueError):
            self.knowledge.observe('direction', 0, 0, bearing_deg=10)

    def test_fallback_point_is_a_surviving_cell(self):
        """后备清除点必须是存活单元，且不能再被提前删除。"""
        self.knowledge.observe('direction', 100, 0, bearing_deg=180)
        point = self.knowledge.fallback_point(np.array([0.0, 5.0]))
        self.assertIsNotNone(point)
        self.assertTrue(np.any(np.all(self.knowledge.possible_points() == point, axis=1)))

    def test_planned_hypotheses_stay_inside_declared_bins(self):
        """启发式样本必须落在声明的半径与朝向区间内，且数量受限。"""
        self.knowledge.observe('direction', 100, 0, bearing_deg=180)
        rows = self.knowledge.planning_hypotheses(8)
        self.assertLessEqual(len(rows), 8)
        self.assertEqual(rows.shape[1], 5)
        self.assertTrue(np.all(rows[:, 2] >= 0))
        self.assertTrue(np.all((rows[:, 4] >= 0) & (rows[:, 4] < 360)))

    def test_reliable_clear_requires_distance_bound_over_all_cells(self):
        """可靠清除要覆盖全部存活单元，而不是只看最近的格点。"""
        self.knowledge.observe('direction', 100, 0, bearing_deg=180)
        cell = self.knowledge.fallback_point(np.array([0.0, 0.0]))
        self.assertFalse(self.knowledge.certain_clear(float(cell[0]), float(cell[1])))


class ChannelSelectionTests(unittest.TestCase):
    """调度必须服务虚拟时间（移动占九成），同时保证没有频道被饿死。"""

    TRAVEL = {3: 900.0, 7: 100.0, 11: 400.0}

    def setUp(self):
        """构造只测试选择规则的策略；不执行任何动作。"""
        self.strategy = Problem4Strategy(MemoryContext())
        self.strategy._channel_travel_m = lambda channel: self.TRAVEL[channel]

    def test_prefers_the_channel_with_the_nearest_next_target(self):
        """未超龄时选下一个追踪点最近的频道，以压低行进（虚拟时间第一成本项）。"""
        self.strategy.round = 5
        self.strategy.last_served.update({3: 4, 7: 4, 11: 4})
        self.assertEqual(self.strategy._next_channel([3, 7, 11]), 7)

    def test_aged_channel_is_forced_ahead_of_nearer_ones(self):
        """等待超过公平年龄的频道必须被强制优先，避免被"最近优先"永久饿死。"""
        self.strategy.round = 5 + self.strategy.config.fairness_age_rounds
        self.strategy.last_served.update({3: 5, 7: 20, 11: 9})
        self.assertEqual(self.strategy._next_channel([3, 7, 11]), 3)

    def test_oldest_of_the_aged_channels_wins(self):
        """多个超龄频道时服务等待最久的那个，保证等待时间有上界。"""
        self.strategy.round = 400
        self.strategy.last_served.update({3: 40, 7: 30, 11: 20})
        self.assertEqual(self.strategy._next_channel([3, 7, 11]), 11)

    def test_selection_only_orders_channels(self):
        """选择规则只排序，不改位置与状态；用于确认它不引入额外几何假设。"""
        self.strategy.round = 5
        self.strategy.last_served.update({3: 4, 7: 4, 11: 4})
        before = self.strategy.position.copy()
        self.strategy._next_channel([3, 7, 11])
        np.testing.assert_array_equal(before, self.strategy.position)


class AdaptiveTrackingTests(unittest.TestCase):
    """自适应追踪轮数必须按存活单元数缩放，且关闭时完全等于原规则。"""

    def setUp(self):
        """构造策略与两类频道：少数存活单元与全量未初始化单元。"""
        self.strategy = Problem4Strategy(MemoryContext())
        self.few = self.strategy.channels[1]
        self.few.observe('near', 30.0, 0.0)
        self.many = self.strategy.channels[2]

    def test_disabled_adaptive_keeps_base_limit(self):
        """两个阈值都为0时返回配置的基础轮数，不看单元数。"""
        config = Problem4Config(adaptive_units_low=0, adaptive_units_high=0)
        self.strategy.config = config
        self.assertEqual(self.strategy._tracking_limit_for(self.few), 2)
        self.assertEqual(self.strategy._tracking_limit_for(self.many), 2)

    def test_few_surviving_units_skip_tracking(self):
        """单元数不超过下阈值的频道直接转入后备清除，不再花检测轮。"""
        config = Problem4Config(adaptive_units_low=12, adaptive_units_high=0)
        self.strategy.config = config
        self.assertEqual(self.strategy._tracking_limit_for(self.few), 0)

    def test_many_surviving_units_get_extra_rounds(self):
        """单元数达到上阈值的频道获得额外追踪轮数以整片收缩区域。"""
        config = Problem4Config(adaptive_units_low=0, adaptive_units_high=25,
                                adaptive_extra_rounds=2)
        self.strategy.config = config
        self.assertGreaterEqual(len(self.many.possible_points()), 25)
        self.assertEqual(self.strategy._tracking_limit_for(self.many), 4)
        self.assertEqual(self.strategy._tracking_limit_for(self.few), 2)

    def test_low_threshold_plan_goes_straight_to_fallback(self):
        """下阈值命中时追踪计划就是后备清除，而非继续示向追击。"""
        config = Problem4Config(adaptive_units_low=12, adaptive_units_high=0)
        self.strategy.config = config
        self.strategy.channels[1] = self.few
        plan = self.strategy._tracking_plan(1)
        self.assertEqual(plan.kind, 'fallback_clear')
        self.assertEqual(plan.channel, 1)

    def test_adaptive_thresholds_must_be_nonnegative_integers(self):
        """阈值带布尔或负值时在配置阶段拒绝。"""
        for bad in (-1, True):
            with self.assertRaises(ValueError):
                Problem4Config(adaptive_units_low=bad)
            with self.assertRaises(ValueError):
                Problem4Config(adaptive_units_high=bad)
            with self.assertRaises(ValueError):
                Problem4Config(adaptive_extra_rounds=bad)

    def test_secondary_score_weights_must_be_valid(self):
        """共享信息增益权重与shortlist因子非法时配置阶段拒绝。"""
        with self.assertRaises(ValueError):
            Problem4Config(shared_gain_weight_s=-5.0)
        with self.assertRaises(ValueError):
            Problem4Config(shortlist_radius_factor=0.9)

    def test_search_layout_must_be_known(self):
        """布站设计只允许两种实现；三角格口径下边长上限仍然生效。"""
        with self.assertRaises(ValueError):
            Problem4Config(search_layout='spiral')
        with self.assertRaises(ValueError):
            Problem4Config(search_layout='triangular', search_spacing_m=1001)
        self.assertEqual(Problem4Config(search_spacing_m=1001).search_layout, 'optimized')


class SearchCertificateTests(unittest.TestCase):
    """搜索布站必须同时给出"任意朝向可发现"与"满覆盖可排除"两项保证。

    解析三角格与启发式优化布站都要过同一组判据；顶点数是排除证据的
    动作数下限，优化布站的收益由 test_waypoint_count_documents_sweep_cost 锁定。
    """

    @classmethod
    def setUpClass(cls):
        """预生成两套搜索顶点与区域内采样网格，供各项证书共用。"""
        cls.layouts = {'triangular900': triangular_search_waypoints(900.0),
                       'optimized': optimized_search_waypoints()}
        step = 300.0
        axis = np.arange(-TARGET_RADIUS_M, TARGET_RADIUS_M + step, step)
        cls.samples = np.array([(x, y) for x in axis for y in axis
                                if np.hypot(x, y) <= TARGET_RADIUS_M + 1e-9])

    def test_covering_radius_keeps_margin_below_min_receive_radius(self):
        """区域上每个目标到最近顶点的距离必须小于最小有效接收半径。"""
        for name, points in self.layouts.items():
            with self.subTest(layout=name):
                worst = 0.0
                for query in self.samples:
                    worst = max(worst, float(np.linalg.norm(points - query, axis=1).min()))
                self.assertLessEqual(worst, 900.0 + 1e-6)
                self.assertLess(worst, MIN_RECEIVE_RADIUS_M - 90.0)

    def test_every_position_and_orientation_can_be_seen(self):
        """目标位于若干1000米内顶点的凸包内，则任一朝向必被至少一个顶点正面看到。

        若方向集合的环形最大空隙不超过180度，0必在其凸包中，等价于不存在
        使全部顶点落在背面的朝向。定向源的"两侧各90度"由此得到任意朝向保证。
        """
        for name, points in self.layouts.items():
            with self.subTest(layout=name):
                for query in self.samples:
                    nearby = points[np.linalg.norm(points - query, axis=1)
                                    <= MIN_RECEIVE_RADIUS_M + 1e-9]
                    self.assertGreater(len(nearby), 0)
                    offset = nearby - query
                    nonzero = np.linalg.norm(offset, axis=1) > 1e-9
                    if not nonzero.any():
                        continue  # 目标恰落在顶点上：零距离对任意朝向都算正面
                    angles = np.sort(np.arctan2(offset[nonzero, 1], offset[nonzero, 0]))
                    gaps = np.diff(np.concatenate((angles, [angles[0] + 2 * np.pi])))
                    self.assertLessEqual(float(gaps.max()), np.pi + 1e-9)

    def test_waypoints_stay_within_confirmed_docking_range(self):
        """顶点必须在演练已证实可停靠的半径内，且互相不重合。"""
        for name, points in self.layouts.items():
            with self.subTest(layout=name):
                self.assertLessEqual(float(np.linalg.norm(points, axis=1).max()),
                                     TARGET_RADIUS_M + 900.0 + 1e-6)
                offsets = points[:, None, :] - points[None, :, :]
                offsets[np.arange(len(points)), np.arange(len(points))] = np.inf
                self.assertGreaterEqual(float(np.linalg.norm(offsets, axis=2).min()), 1.0)

    def test_waypoint_count_documents_sweep_cost(self):
        """顶点个数是排除证据的动作数下限（每频道需遍历全部顶点）。

        三角格数字由格距与目标半径决定；优化布站必须真的更少才有意义，
        两者的调整都必须同时复核上面的覆盖与包围证书。
        """
        self.assertEqual(len(self.layouts['triangular900']), 37)
        radii = np.linalg.norm(self.layouts['triangular900'], axis=1)
        self.assertLessEqual(float(radii.max()), TARGET_RADIUS_M + 900.0 + 1e-6)
        self.assertGreater(float(radii.max()), TARGET_RADIUS_M)
        optimized = self.layouts['optimized']
        self.assertLess(len(optimized), 37)


class ResultAndSummaryTests(unittest.TestCase):
    """结果字段必须能与问题三同一口径判读，异常与未证明不得混同。"""

    def setUp(self):
        """初始化不允许动作的上下文与策略。"""
        self.strategy = Problem4Strategy(MemoryContext())

    def test_result_exposes_evidence_and_anomaly_fields(self):
        """未开始时全部频道未证明，且不出现模型异常。"""
        result = self.strategy.result()
        self.assertFalse(result['completed'])
        self.assertEqual(result['unresolved_channels'], list(range(1, 21)))
        self.assertEqual(result['inconsistent_channels'], [])
        self.assertEqual(result['search_waypoints'], len(self.strategy.waypoints))
        self.assertEqual(sorted(result['channel_series']), list(range(1, 21)))
        self.assertEqual(result['cleared_channels'], [])

    def test_unresolved_and_inconsistent_are_reported_separately(self):
        """异常频道既不算已清除，也不再算未证明。"""
        self.strategy.channels[1].mark_cleared()
        self.strategy.channels[2].status = 'inconsistent'
        result = self.strategy.result()
        self.assertEqual(result['cleared_channels'], [1])
        self.assertEqual(result['inconsistent_channels'], [2])
        self.assertEqual(result['unresolved_channels'], list(range(3, 21)))
        self.assertEqual(result['planner_errors'], 0)

    def test_summary_keeps_unresolved_under_cleared_limit(self):
        """清满上界收工时，未证明频道应保留在汇总里而不是被当作不存在。"""
        for channel in range(1, 17):
            self.strategy.channels[channel].mark_cleared()
        self.assertEqual(self.strategy.finished(), 'cleared_limit')
        self.strategy.stop_reason = 'cleared_limit'
        summary = summarize(self.strategy.result(), true_total=16)
        self.assertTrue(summary['completed'])
        self.assertEqual(summary['cleared_count'], 16)
        self.assertEqual(summary['cleared_ratio'], 1.0)
        self.assertEqual(summary['unresolved_channels'], list(range(17, 21)))

    def test_summary_rejects_impossible_true_total(self):
        """真实总数不能小于已清除数，也不能超过频道上限。"""
        self.strategy.channels[1].mark_cleared()
        record = self.strategy.result()
        for bad in (0, -1, 25):
            with self.assertRaises(ValueError):
                summarize(record, true_total=bad)
        summary = summarize(record, true_total=8)
        self.assertEqual(summary['cleared_ratio'], 0.125)

    def test_model_inconsistent_is_not_completion(self):
        """空联合外包必须以异常收尾，不得当成频道不存在。"""
        lattice = Lattice(np.array([[0.0, 0.0]]), 1.0, 1.0, 0.0)
        strategy = Problem4Strategy(MemoryContext())
        strategy.channels[4] = Problem4Knowledge(4, lattice)
        strategy.channels[4].observe('near', 0, 0)
        strategy.channels[4].observe('near', 100, 100)
        self.assertEqual(strategy.finished(), 'model_inconsistent')
        strategy.stop_reason = 'model_inconsistent'
        self.assertFalse(strategy.result()['completed'])
        self.assertEqual(strategy.result()['planner_errors'], 1)


class OfflineEntryTests(unittest.TestCase):
    """命令行入口：不带记录文件时在本地桩上自检，带记录文件时只做汇总。"""

    def test_build_scenario_mixes_requested_share(self):
        """定向源比例可按参数调整，源数不足时退化为全向案例。"""
        self.assertTrue(all(s.direction_deg is None for s in build_scenario(12, 1, 0).sources))
        share = build_scenario(12, 1, None)
        self.assertEqual(len(share.sources), 12)
        self.assertEqual(sum(s.direction_deg is not None for s in share.sources), 6)
        clamped = build_scenario(12, 1, 99)
        self.assertEqual(sum(s.direction_deg is not None for s in clamped.sources), 11)
        self.assertTrue(all(s.direction_deg is None for s in build_scenario(1, 1, None).sources))

    def test_record_mode_only_summarises(self):
        """给出记录文件时不应发起任何请求，只打印统计量。"""
        record = {'cleared_channels': [3], 'start_virtual_time_s': 0.0,
                  'end_virtual_time_s': 120.0, 'real_elapsed_s': 1.5,
                  'stop_reason': 'budget_limit', 'moved_distance_m': 500.0,
                  'planner_errors': 0, 'completed': False,
                  'steps': [{'kind': 'measure', 'result': 'no_signal'}]}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'record.json'
            path.write_text(json.dumps({'record': record}), encoding='utf-8')
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                self.assertIsNone(main([str(path), '--true-total', '2']))
        payload = json.loads(buffer.getvalue())
        self.assertEqual(payload['cleared_count'], 1)
        self.assertEqual(payload['true_total'], 2)
        self.assertEqual(payload['cleared_ratio'], 0.5)
        self.assertEqual(payload['no_signal_count'], 1)
        self.assertFalse(payload['completed'])

    def test_record_mode_rejects_impossible_total(self):
        """汇总入口同样拒绝不可能的真实总数。"""
        record = {'cleared_channels': [1, 2], 'start_virtual_time_s': 0.0,
                  'end_virtual_time_s': 10.0, 'real_elapsed_s': 0.1,
                  'stop_reason': 'budget_limit', 'moved_distance_m': 0.0,
                  'planner_errors': 0, 'completed': False, 'steps': []}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'record.json'
            path.write_text(json.dumps(record), encoding='utf-8')
            with self.assertRaises(ValueError):
                main([str(path), '--true-total', '1'])


class RecordPlacementTests(unittest.TestCase):
    """记录落盘目录：离线自检归 output/protocol，在线演练/正式归 output/ProblemN。"""

    def test_record_dir_follows_the_online_offline_split(self):
        """分工由 config.record_dir_for 统一给出：离线永远进 protocol，未知题号直接拒绝。"""
        self.assertEqual(record_dir_for(4, offline=True), PROTOCOL_LOG_DIR)
        self.assertEqual(record_dir_for(4, offline=False), PROBLEM4_OUTPUT_DIR)
        self.assertEqual(record_dir_for(3, offline=False).name, 'Problem3')
        with self.assertRaises(ValueError):
            record_dir_for(9, offline=False)
        with self.assertRaises(ValueError):
            record_dir_for('四', offline=False)

    def test_only_the_stub_transport_counts_as_offline(self):
        """在线传输必须被判成在线，否则演练成绩会被写进离线目录。"""
        online = StrategyContext(RobotClient('team-x',
                                             HttpTransport('http://127.0.0.1:2026',
                                                           allow_network=True)), 4)
        self.assertFalse(is_offline_run(online))
        self.assertFalse(is_offline_run(SimpleNamespace(client=RobotClient('team-x'))))
        stub = OfflineStub()
        self.assertTrue(is_offline_run(StrategyContext(RobotClient('offline-team', stub), 4)))

    def test_offline_run_writes_its_record_under_protocol(self):
        """离线跑一局：记录必须落在 protocol 且带 offline- 前缀，不污染 output/Problem4。"""
        scenario = fixed_scenario([(1, 300.0, 400.0), (2, -900.0, 800.0)])
        client = RobotClient('offline-team', OfflineStub(sources=scenario.sources))
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch.object(settings, 'PROTOCOL_LOG_DIR', Path(tmp)):
                summary = run_strategy(client, solve, problem=4)['algorithm_result']
            target = Path(summary['record_path'])
            self.assertEqual(target.parent, Path(tmp))
            self.assertTrue(target.name.startswith('offline-mission_p4_'), target.name)
            self.assertTrue(target.exists())
            payload = json.loads(target.read_text(encoding='utf-8'))
            self.assertEqual(payload['summary']['cleared_count'], 2)
            self.assertEqual(payload['record']['problem'], 4)


@unittest.skipUnless(HAVE_MATPLOTLIB, '绘制任务图需要 matplotlib。')
class PlottingTests(unittest.TestCase):
    """任务图使用Agg后端写盘，不弹窗、不联网。"""

    def test_mission_figure_writes_png_and_svg(self):
        """一次小案例的完整记录应能画出四联图。"""
        from .problem4_plotting import plot_mission
        scenario = fixed_scenario([(1, 300.0, 400.0), (2, -900.0, 800.0)])
        client = RobotClient('offline-team', OfflineStub(sources=scenario.sources))
        record = run_strategy(client, run_mission, problem=4)['algorithm_result']
        with tempfile.TemporaryDirectory() as tmp:
            target = plot_mission(record, scenario=scenario, figures_dir=tmp, name='mission_test')
            self.assertTrue(target.exists())
            self.assertGreater(target.stat().st_size, 1000)
            self.assertTrue((Path(tmp) / 'mission_test.svg').exists())

    def test_plot_from_path_needs_a_record(self):
        """没有记录路径时不绘图，也不报错。"""
        from .problem4_plotting import plot_mission_from_path
        self.assertIsNone(plot_mission_from_path(None))

    def test_cli_draws_from_an_existing_record(self):
        """从记录文件出图：演练记录没有真值，也应能画出轨迹与证据三幅。"""
        from .problem4_plotting import main as plot_main
        scenario = fixed_scenario([(1, 300.0, 400.0), (2, -900.0, 800.0)])
        client = RobotClient('offline-team', OfflineStub(sources=scenario.sources))
        record = run_strategy(client, run_mission, problem=4)['algorithm_result']
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'record.json'
            path.write_text(json.dumps({'record': record}), encoding='utf-8')
            target = plot_main([str(path), '--figures-dir', tmp, '--name', 'cli_test'])
            self.assertTrue(Path(target).exists())
            self.assertEqual(Path(target).name, 'cli_test.png')
            self.assertTrue((Path(tmp) / 'cli_test.svg').exists())


@unittest.skipUnless(os.environ.get('RUN_PROBLEM4_E2E') == '1', '完整离线回归须显式启用（默认跳过以缩短常规自检）。')
class EndToEndTests(unittest.TestCase):
    """完整流程回归与边界/混合案例；不生成任何官方成绩。"""

    def _run_case(self, scenario):
        """仅把真值交给桩；策略只接收公开上下文。"""
        stub = OfflineStub(sources=scenario.sources, location_error_deg=0.9)
        client = RobotClient('offline-team', stub)
        result = run_strategy(client, run_mission, problem=4)['algorithm_result']
        self.assertTrue(result['completed'], result['stop_reason'])
        self.assertEqual(set(result['cleared_channels']), set(scenario.channels()))
        return result

    def test_mixed(self):
        """12源混合案例完整清除回归。"""
        self._run_case(mixed_scenario())

    def test_outward_boundary(self):
        """边界朝外源必须通过区域外检测发现。"""
        self._run_case(boundary_scenario())

    def test_omnidirectional_regression(self):
        """问题四策略仍应能够处理纯全向的小型基线。"""
        self._run_case(fixed_scenario([(1, 300, 400), (2, -900, 800)]))

    def test_exclusion_certificate_matches_hidden_truth(self):
        """被判"不存在"的频道与真实源集合必须交集为空，既不能漏判也不能误判。"""
        from .scenario import random_scenario
        scenario = random_scenario(seed=1, n_sources=10)
        result = self._run_case(scenario)
        excluded = {channel for channel, status in result['channels'].items()
                    if status['status'] == 'excluded'}
        self.assertEqual(excluded & set(scenario.channels()), set())
        self.assertEqual(len(excluded), 10)
        for channel in excluded:
            self.assertEqual(result['channels'][channel]['coverage_done'],
                             result['channels'][channel]['coverage_required'])
        self.assertEqual(result['stop_reason'], 'all_channels_resolved')

    def test_exhausted_budget_is_not_completion(self):
        """真实预算耗尽时必须安全收尾，且不得声称完成或发新动作。"""
        scenario = fixed_scenario([(1, 300, 400), (2, -900, 800)])
        client = RobotClient('offline-team', OfflineStub(sources=scenario.sources, remaining=5))
        result = run_strategy(client, run_mission, problem=4)['algorithm_result']
        self.assertFalse(result['completed'])
        self.assertEqual(result['stop_reason'], 'budget_limit')
        self.assertEqual(result['steps'], [])


if __name__ == '__main__':
    unittest.main()
