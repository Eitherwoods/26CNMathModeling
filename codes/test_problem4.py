# -*- coding: utf-8 -*-
"""问题四待执行测试脚手架：几何、联合外包、动作与完成证据。

本文件创建时未运行。普通unittest只用内存上下文/本地桩；较长端到端用例
需显式设置RUN_PROBLEM4_E2E=1。任何用例都不会连接官方模拟器。
"""
import os
import unittest
from types import SimpleNamespace

import numpy as np

from .offline_stub import OfflineStub, Source
from .problem3_model import Lattice
from .problem4_model import Problem4Knowledge, triangular_search_waypoints
from .problem4_solution import Problem4Config, Problem4Strategy, feedback_score, run_mission
from .protocol import RobotClient
from .scenario import fixed_scenario
from .scenario_p4 import boundary_scenario, mixed_scenario
from .strategy import BudgetReached, run_strategy


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


@unittest.skipUnless(os.environ.get('RUN_PROBLEM4_E2E') == '1', '完整离线回归须显式启用；当前尚未执行。')
class EndToEndTests(unittest.TestCase):
    """完整流程回归与边界/混合案例；不生成任何官方成绩。"""

    def _run_case(self, scenario):
        """仅把真值交给桩；策略只接收公开上下文。"""
        stub = OfflineStub(sources=scenario.sources, location_error_deg=0.9)
        client = RobotClient('offline-team', stub)
        result = run_strategy(client, run_mission, problem=4)['algorithm_result']
        self.assertTrue(result['completed'], result['stop_reason'])
        self.assertEqual(set(result['cleared_channels']), set(scenario.channels()))

    def test_mixed(self):
        """12源混合案例完整清除回归。"""
        self._run_case(mixed_scenario())

    def test_outward_boundary(self):
        """边界朝外源必须通过区域外检测发现。"""
        self._run_case(boundary_scenario())

    def test_omnidirectional_regression(self):
        """问题四策略仍应能够处理纯全向的小型基线。"""
        self._run_case(fixed_scenario([(1, 300, 400), (2, -900, 800)]))


if __name__ == '__main__':
    unittest.main()
