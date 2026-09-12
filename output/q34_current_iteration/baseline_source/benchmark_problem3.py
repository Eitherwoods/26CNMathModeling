# -*- coding: utf-8 -*-
"""问题三调度策略的可重复离线对照基准，不连接官方模拟器。"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import replace

from .offline_stub import OfflineStub
from .problem3_solution import Problem3Config, run_mission
from .protocol import RobotClient
from .scenario import random_scenario
from .strategy import run_strategy


def benchmark_cases():
    """返回覆盖不同源数和分布的固定案例集。"""
    cases = [random_scenario(seed=seed, n_sources=n)
             for n in (10, 12, 14, 16)
             for seed in range(1, 3)]
    return cases


def run_case(scenario, config):
    """在同一离线桩口径下运行单个案例并提取虚拟时间指标。"""
    stub = OfflineStub(sources=scenario.sources)
    client = RobotClient('offline-team', stub)
    solver = lambda context: run_mission(context, config)
    record = run_strategy(client, solver, problem=3)['algorithm_result']
    return {'name': scenario.name, 'completed': record['stop_reason'] in ('all_channels_resolved', 'cleared_limit'),
            'cleared': len(record['cleared_channels']),
            'true_total': scenario.true_total,
            'virtual_time_s': record['end_virtual_time_s'] - record['start_virtual_time_s'],
            'moved_distance_m': record['moved_distance_m'], 'actions': len(record['steps']),
            'stop_reason': record['stop_reason']}


def compare_configs(cases=None):
    """比较基线与 Dijkstra 路线变体，返回逐案例和合计指标。"""
    cases = list(cases or benchmark_cases())
    base = Problem3Config()
    variants = {
        'baseline': base,
        'dijkstra_route': replace(base, route_planner='dijkstra',
                                   enable_route_aware_endgame=True,
                                   route_endgame_min_clears=8),
        'intermediate_stops': replace(base, intermediate_stop_gap_m=800.0,
                                       enable_route_aware_endgame=True,
                                       route_endgame_min_clears=8),
        'combined': replace(base, route_planner='dijkstra',
                             intermediate_stop_gap_m=800.0,
                             enable_route_aware_endgame=True,
                             route_endgame_min_clears=8),
    }
    output = {'cases': len(cases), 'variants': {}}
    for name, config in variants.items():
        started = time.perf_counter()
        rows = [run_case(case, config) for case in cases]
        output['variants'][name] = {
            'completed': sum(row['completed'] for row in rows),
            'total_virtual_time_s': sum(row['virtual_time_s'] for row in rows),
            'total_moved_distance_m': sum(row['moved_distance_m'] for row in rows),
            'total_actions': sum(row['actions'] for row in rows),
            'wall_time_s': time.perf_counter() - started,
            'results': rows,
        }
    base_total = output['variants']['baseline']['total_virtual_time_s']
    for name in ('dijkstra_route', 'intermediate_stops', 'combined'):
        total = output['variants'][name]['total_virtual_time_s']
        output['variants'][name]['virtual_time_reduction'] = None if base_total == 0 else 1 - total / base_total
    return output


def main(argv=None):
    """命令行运行固定案例集并打印 JSON，便于保存参数对照证据。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args(argv)
    print(json.dumps(compare_configs(), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
