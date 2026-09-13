# -*- coding: utf-8 -*-
"""Route-First 策略（problem3_route）的单元与离线端到端测试。"""
from __future__ import annotations

import unittest

import numpy as np

from .offline_stub import OfflineStub
from .problem3_route import RouteFirstStrategy, run_route_mission
from .problem3_solution import Problem3Config, run_mission
from .protocol import RobotClient
from .scenario import random_scenario, fixed_scenario
from .strategy import run_strategy


class _DummyContext:
    """仅提供初始位置的桩上下文（不发送任何请求）。"""

    def __init__(self):
        from types import SimpleNamespace
        self.state = SimpleNamespace(position=(0.0, 0.0), channel=1,
                                     virtual_time_s=0.0, cleared_channels=frozenset())
        self.problem = 3


class TourPlanTests(unittest.TestCase):
    """巡游构造：站点全覆盖、起点正确、2-opt 不重不漏。"""

    def setUp(self):
        self.strategy = RouteFirstStrategy(_DummyContext(), Problem3Config())

    def test_tour_covers_all_stations_exactly_once(self):
        queue = np.asarray(self.strategy.tour_queue, dtype=float)
        stations = np.asarray(self.strategy.tour_stations, dtype=float)
        self.assertEqual(len(queue), len(stations))
        ordered = queue[np.lexsort((queue[:, 1], queue[:, 0]))]
        reference = stations[np.lexsort((stations[:, 1], stations[:, 0]))]
        self.assertTrue(np.allclose(ordered, reference))

    def test_tour_starts_nearest_to_origin(self):
        first = np.asarray(self.strategy.tour_queue[0], dtype=float)
        # 原点本身就是候选站点，最近邻必从原点出发。
        self.assertAlmostEqual(float(np.hypot(*first)), 0.0, places=6)

    def test_grid9_tour_uses_certificate_stations(self):
        config = Problem3Config(route_tour_set='grid9')
        strategy = RouteFirstStrategy(_DummyContext(), config)
        self.assertEqual(len(strategy.tour_stations), 9)


class ModeDispatchTests(unittest.TestCase):
    """run_mission 按 strategy_mode 分派；route 模式结果可复现。"""

    def test_mpc_is_default(self):
        case = fixed_scenario([(1, 900.0, 0.0), (2, -700.0, 600.0),
                               (3, 500.0, -800.0)])
        stub = OfflineStub(sources=case.sources)
        client = RobotClient('offline-team', stub)
        record = run_strategy(client, lambda ctx: run_mission(ctx), problem=3)
        self.assertIn(record['algorithm_result']['stop_reason'],
                      ('all_channels_resolved', 'cleared_limit'))

    def test_route_mode_end_to_end(self):
        case = fixed_scenario([(1, 900.0, 0.0), (2, -700.0, 600.0),
                               (3, 500.0, -800.0)])
        stub = OfflineStub(sources=case.sources)
        client = RobotClient('offline-team', stub)
        record = run_strategy(
            client, lambda ctx: run_mission(ctx, Problem3Config(strategy_mode='route')),
            problem=3)['algorithm_result']
        self.assertIn(record['stop_reason'],
                      ('all_channels_resolved', 'cleared_limit'))
        self.assertEqual(record['planner_errors'], 0)

    def test_route_run_route_mission_direct(self):
        case = random_scenario(seed=1, n_sources=10)
        stub = OfflineStub(sources=case.sources)
        client = RobotClient('offline-team', stub)
        record = run_strategy(
            client, lambda ctx: run_route_mission(ctx, Problem3Config()),
            problem=3)['algorithm_result']
        self.assertIn(record['stop_reason'],
                      ('all_channels_resolved', 'cleared_limit'))


class RouteBenchmarkSmokeTests(unittest.TestCase):
    """Route-First 在随机案例上完成全部清除（较慢，默认跳过）。"""

    def test_random_case_completed(self):
        case = random_scenario(seed=1, n_sources=12)
        stub = OfflineStub(sources=case.sources)
        client = RobotClient('offline-team', stub)
        record = run_strategy(
            client, lambda ctx: run_mission(ctx, Problem3Config(strategy_mode='route')),
            problem=3)['algorithm_result']
        self.assertEqual(len(record['cleared_channels']), 12)


if __name__ == '__main__':
    unittest.main()
