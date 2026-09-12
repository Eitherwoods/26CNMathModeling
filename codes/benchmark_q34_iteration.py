# -*- coding: utf-8 -*-
"""q3/q4 统一离线迭代基准工具。

该模块只调用现有场景、OfflineStub、run_strategy 和 run_mission，不改变求解器。
它保存逐例原始记录、配置、场景真值和协议/轨迹核验，便于不同逻辑变更做公平对照。
"""
from __future__ import annotations

import argparse
import dataclasses
import hashlib
import importlib.metadata
import json
import math
import platform
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path

from .offline_stub import OfflineStub
from .problem3_solution import Problem3Config, run_mission as run_mission3
from .problem4_solution import Problem4Config, run_mission as run_mission4
from .protocol import RobotClient
from .scenario import random_scenario
from .scenario_p4 import mixed_scenario
from .strategy import run_strategy


def _jsonable(value):
    """将 numpy、dataclass、集合和有限记录递归转换为可写 JSON 的值。"""
    if dataclasses.is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(v) for v in value]
    if hasattr(value, 'item'):
        return _jsonable(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _git_hash():
    """返回当前代码提交哈希；非 git 环境返回 unknown。"""
    try:
        return subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True,
                                       stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'


def _code_hashes():
    """记录 codes 目录中 Python 源文件哈希，区分未提交修改。"""
    root = Path(__file__).resolve().parent
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(root.glob('*.py'))}


def _config(problem, overrides):
    """按问题构造配置，并仅接受对应 dataclass 已存在的字段。"""
    cls = Problem3Config if problem == 3 else Problem4Config
    fields = {f.name for f in dataclasses.fields(cls)}
    unknown = set(overrides) - fields
    if unknown:
        raise ValueError(f'problem {problem} unknown config fields: {sorted(unknown)}')
    # JSON 数组可作为原配置的 tuple 参数（如 lookahead_lengths_m）。
    values = dict(overrides)
    for f in dataclasses.fields(cls):
        if f.name in values and f.type == tuple and isinstance(values[f.name], list):
            values[f.name] = tuple(values[f.name])
    return cls(**values)


def _scenario(problem, seed, sources):
    """按统一基准口径生成确定性场景。"""
    return (random_scenario(seed=seed, n_sources=sources)
            if problem == 3 else mixed_scenario(seed=seed, n_sources=sources,
                                                n_directional=max(1, sources // 2)))


def _check(record, scenario, client, stub):
    """核验结果合法性、轨迹连续性、协议边界和 OfflineStub 计时一致性。"""
    rec = record or {}
    steps = rec.get('steps') or []
    trajectory = rec.get('trajectory') or []
    channels = rec.get('cleared_channels') or []
    finite_coords = True
    for point in trajectory:
        try:
            finite_coords &= len(point) == 2 and all(math.isfinite(float(v)) and abs(float(v)) <= 2_000_000 for v in point)
        except (TypeError, ValueError):
            finite_coords = False
    step_channels = []
    for step in steps:
        if isinstance(step, dict) and step.get('channel') is not None:
            step_channels.append(step['channel'])
    channels_legal = all(type(c) is int and 1 <= c <= 20 for c in channels)
    unique_clear = len(channels) == len(set(channels))
    trajectory_steps = len(trajectory) == len(steps) + 1 if steps else len(trajectory) in (0, 1)
    for index, step in enumerate(steps):
        try:
            trajectory_steps &= (math.isclose(float(step['x']), float(trajectory[index + 1][0]), abs_tol=1e-7)
                                 and math.isclose(float(step['y']), float(trajectory[index + 1][1]), abs_tol=1e-7))
        except (KeyError, TypeError, ValueError, IndexError):
            trajectory_steps = False
    distance = 0.0
    try:
        distance = sum(math.dist(trajectory[i - 1], trajectory[i]) for i in range(1, len(trajectory)))
    except (TypeError, ValueError):
        trajectory_steps = False
    moved = rec.get('moved_distance_m')
    distance_match = moved is None or math.isclose(float(moved), distance, rel_tol=1e-6, abs_tol=1e-3)
    start, end = rec.get('start_virtual_time_s'), rec.get('end_virtual_time_s')
    record_time = None if start is None or end is None else float(end) - float(start)
    stub_time = float(client.state.virtual_time_s)
    time_match = record_time is None or math.isclose(record_time, stub_time, rel_tol=1e-9, abs_tol=1e-6)
    recomputed_us, previous_channel = 0, 1
    action_time_match = True
    clear_truth_match = True
    for index, step in enumerate(steps):
        try:
            point, previous = trajectory[index + 1], trajectory[index]
            increment = round(math.dist(previous, point) / 5.0 * 1e6)
            kind, channel = step['kind'], step.get('channel')
            if kind == 'measure':
                increment += 5_000_000 + (1_000_000 if channel != previous_channel else 0)
                previous_channel = channel
            elif kind == 'clear':
                success = step.get('result') == 'success'
                increment += 5_000_000 if success else 3_000_000
                source = stub.sources.get(channel)
                clear_truth_match &= success == (source is not None and math.dist(point, (source.x, source.y)) <= 20)
            else:
                action_time_match = False
                continue
            recomputed_us += increment
            action_time_match &= math.isclose(float(step.get('virtual_time_s', -1)) * 1e6,
                                               recomputed_us, abs_tol=1.5)
        except (KeyError, TypeError, ValueError, IndexError):
            action_time_match = False
    action_time_match &= math.isclose(recomputed_us / 1e6, stub_time, abs_tol=1e-6)
    true_channels = {source.channel for source in scenario.sources}
    truth_match = set(channels) == (set(channels) & true_channels)
    stop_reason = rec.get('stop_reason')
    derived_completed = stop_reason in ('all_channels_resolved', 'cleared_limit')
    completed_consistent = ('completed' not in rec or bool(rec.get('completed')) == derived_completed)
    complete_valid = (derived_completed and completed_consistent and len(channels) == scenario.true_total
                      and set(channels) == true_channels)
    protocol_boundary = all(type(c) is int and 1 <= c <= 20 for c in step_channels) and finite_coords
    return {
        'channels_legal': channels_legal, 'cleared_unique': unique_clear,
        'trajectory_steps_match': trajectory_steps, 'distance_match': distance_match,
        'stub_time_match': time_match, 'action_time_match': action_time_match,
        'clear_truth_match': clear_truth_match, 'cleared_truth_match': truth_match,
        'complete_valid': complete_valid, 'finite_coordinates': finite_coords,
        'protocol_boundary': protocol_boundary,
        'planner_errors': int(rec.get('planner_errors', 0) or 0),
        'inconsistent': rec.get('inconsistent_channels', []),
        'all_constraints_ok': all((channels_legal, unique_clear, trajectory_steps,
                                   distance_match, time_match, action_time_match,
                                   clear_truth_match, truth_match, complete_valid,
                                   finite_coords, protocol_boundary,
                                   not rec.get('planner_errors', 0),
                                   not rec.get('inconsistent_channels', []))),
    }


def run_case(problem, seed, sources, overrides):
    """运行单例并返回可复现的原始记录和指标。"""
    scenario = _scenario(problem, seed, sources)
    config = _config(problem, overrides)
    stub = OfflineStub(sources=scenario.sources, location_error_deg=0.9)
    client = RobotClient('offline-team', stub)
    solver = lambda ctx: (run_mission3(ctx, config) if problem == 3 else run_mission4(ctx, config))
    started = time.perf_counter()
    error = None
    try:
        result = run_strategy(client, solver, problem=problem)
        record = result.get('algorithm_result') or {}
    except Exception as exc:  # 保留异常输入/空解证据，继续后续案例
        result, record = {}, {}
        error = {'type': type(exc).__name__, 'message': str(exc)}
    elapsed = time.perf_counter() - started
    check = _check(record, scenario, client, stub) if error is None else {'all_constraints_ok': False, 'error': error}
    virtual = float(client.state.virtual_time_s)
    cleared = len(client.state.cleared_channels)
    return {'seed': seed, 'problem': problem, 'scenario': _jsonable(scenario),
            'config': _jsonable(config), 'record': _jsonable(record),
            'runner_result': _jsonable(result), 'wall_time_s': elapsed,
            'virtual_time_s': virtual, 'cleared': cleared,
            'true_total': scenario.true_total,
            'completion_rate': cleared / scenario.true_total if scenario.true_total else 0.0,
            'checks': _jsonable(check), 'error': error}


def main(argv=None):
    """运行 q3/q4 案例集，逐例 flush 摘要并写入 UTF-8 JSON。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--problem', type=int, choices=(3, 4), required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[1, 2])
    parser.add_argument('--sources', type=int, default=12)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--overrides', default='{}', help='配置字段 JSON 对象')
    args = parser.parse_args(argv)
    overrides = json.loads(args.overrides)
    if not isinstance(overrides, dict):
        raise ValueError('--overrides 必须是 JSON 对象')
    metadata = {'problem': args.problem, 'seeds': args.seeds, 'sources': args.sources,
                'overrides': overrides, 'git_hash': _git_hash(), 'code_sha256': _code_hashes(),
                'python': sys.version, 'platform': platform.platform(),
                'dependencies': {'numpy': importlib.metadata.version('numpy')}}
    output = {'metadata': metadata, 'cases': []}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for seed in args.seeds:
        row = run_case(args.problem, seed, args.sources, overrides)
        output['cases'].append(row)
        with args.output.open('w', encoding='utf-8') as handle:
            json.dump(_jsonable(output), handle, ensure_ascii=False, indent=2)
            handle.flush()
        print(json.dumps({'seed': seed, 'virtual_time_s': row['virtual_time_s'],
                          'cleared': row['cleared'], 'true_total': row['true_total'],
                          'completion_rate': row['completion_rate'],
                          'checks': row['checks']}, ensure_ascii=False), flush=True)
    return output


if __name__ == '__main__':
    main()
