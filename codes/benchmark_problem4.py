# -*- coding: utf-8 -*-
"""问题四调度策略的可重复离线对照基准，不连接官方模拟器。"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import replace

from .offline_stub import OfflineStub
from .problem4_solution import Problem4Config, run_mission
from .protocol import RobotClient
from .scenario import random_scenario
from .scenario_p4 import boundary_scenario, mixed_scenario
from .strategy import run_strategy


def benchmark_cases():
    """返回覆盖混合源、全向源、目标数上下界和边界朝外源的固定案例集。"""
    cases = [mixed_scenario(seed=seed, n_sources=12, n_directional=6)
             for seed in range(1, 4)]
    cases.extend(random_scenario(seed=seed, n_sources=16) for seed in range(1, 6))
    cases.extend(random_scenario(seed=seed, n_sources=10) for seed in range(1, 3))
    cases.append(boundary_scenario())
    return cases


def run_case(scenario, config):
    """在同一离线桩口径下运行单个案例并提取虚拟时间指标。"""
    stub = OfflineStub(sources=scenario.sources, location_error_deg=0.9)
    client = RobotClient('offline-team', stub)
    solver = lambda context: run_mission(context, config)
    record = run_strategy(client, solver, problem=4)['algorithm_result']
    return {'name': scenario.name, 'completed': record['completed'],
            'cleared': len(record['cleared_channels']),
            'virtual_time_s': record['end_virtual_time_s'] - record['start_virtual_time_s'],
            'moved_distance_m': record['moved_distance_m'], 'actions': len(record['steps']),
            'stop_reason': record['stop_reason']}


def compare_configs(cases=None):
    """比较分离路线基线、联合后备清除路线和连续追踪消融结果。

    分离路线固定关闭联合路线，保证对照不会随着默认配置漂移；主比较为
    ``periodic_search`` 与 ``joint_fallback``，``finish_detected`` 仅保留作
    调度消融。所有结果均来自 OfflineStub，不能解释为正式演练成绩。
    """
    cases = list(cases or benchmark_cases())
    base = Problem4Config()
    variants = {
        'periodic_search': replace(base, joint_route_with_clear=False,
                                   joint_route_with_track=False,
                                   finish_detected_before_search=False),
        'joint_fallback': replace(base, joint_route_with_clear=True,
                                   joint_route_with_track=False,
                                   finish_detected_before_search=False),
        'finish_detected': replace(base, joint_route_with_clear=False,
                                   joint_route_with_track=False,
                                   finish_detected_before_search=True),
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
    old = output['variants']['periodic_search']['total_virtual_time_s']
    new = output['variants']['joint_fallback']['total_virtual_time_s']
    output['virtual_time_reduction'] = None if old == 0 else 1 - new / old
    old_distance = output['variants']['periodic_search']['total_moved_distance_m']
    new_distance = output['variants']['joint_fallback']['total_moved_distance_m']
    output['distance_reduction'] = None if old_distance == 0 else 1 - new_distance / old_distance
    return output


def main(argv=None):
    """命令行运行固定案例集并打印JSON，便于保存参数对照证据。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args(argv)
    print(json.dumps(compare_configs(), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
