# -*- coding: utf-8 -*-
"""问题四虚拟时间成本分解诊断。

把每一局的停靠段按 plan.kind 归类（initial / search / track /
reliable_clear / fallback_clear / lls_probe），逐段统计移动距离与动作数，
换算成虚拟时间构成。用于回答"改动到底省在哪"，而不是只看总时间。

用法：
    python -m codes.p4_diagnose --seeds 1 2 3 --output output/protocol/p4_diag.json
    python -m codes.p4_diagnose --seeds 1 --overrides '{"search_layout":"optimized"}'
"""
from __future__ import annotations
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ.setdefault(_v, '1')

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from .benchmark_q34_local import EngineTransport, generate_scenario
from .protocol import RobotClient
from .strategy import run_strategy
from . import config as settings
from .problem4_solution import Problem4Config, Problem4Strategy, run_mission as run4

SEARCH_KINDS = ('initial', 'search')
TRACK_KINDS = ('track',)
CLEAR_KINDS = ('reliable_clear', 'fallback_clear', 'lls_probe')

_DIAG = {'segments': []}


def _install_probe():
    """在停靠段级别记录 plan.kind、移动距离与动作构成。"""
    if getattr(Problem4Strategy, '_diag_installed', False):
        return
    original_measures = Problem4Strategy.execute_measures
    original_clears = Problem4Strategy.execute_clears

    def wrapped_measures(self, plan):
        before = np.array(self.position, dtype=float)
        index = len(self.steps)
        result = original_measures(self, plan)
        _record(self, plan, before, index)
        return result

    def wrapped_clears(self, plan):
        before = np.array(self.position, dtype=float)
        index = len(self.steps)
        result = original_clears(self, plan)
        _record(self, plan, before, index)
        return result

    def _record(self, plan, before, index):
        after = np.array(self.position, dtype=float)
        new_steps = self.steps[index:]
        measures = sum(1 for s in new_steps if s['kind'] == 'measure')
        clears = sum(1 for s in new_steps if s['kind'] == 'clear')
        _DIAG['segments'].append({
            'kind': plan.kind,
            'channel': plan.channel,
            'moved_m': float(np.linalg.norm(after - before)),
            'measures': measures,
            'clears': clears,
            'from': [float(before[0]), float(before[1])],
            'to': [float(after[0]), float(after[1])],
        })

    Problem4Strategy.execute_measures = wrapped_measures
    Problem4Strategy.execute_clears = wrapped_clears
    Problem4Strategy._diag_installed = True


def group_of(kind):
    if kind in SEARCH_KINDS:
        return 'search'
    if kind in TRACK_KINDS:
        return 'track'
    if kind in CLEAR_KINDS:
        return 'clear'
    return 'other'


def run_one(seed, overrides):
    _DIAG['segments'] = []
    scenario = generate_scenario(4, seed)
    transport = EngineTransport(scenario)
    client = RobotClient('p4-diagnose', transport)
    cfg = Problem4Config(**overrides)
    try:
        result = run_strategy(client, lambda ctx: run4(ctx, cfg), problem=4)
    except Exception as exc:  # noqa: BLE001
        return {'seed': seed, 'error': f'{type(exc).__name__}: {exc}'}
    rec = result.get('algorithm_result') or {}
    steps = rec.get('steps') or []

    # 频道切换次数：measure 动作的目标频道与其前一次 different 时计 1 s。
    switches = 0
    previous = None
    for step in steps:
        if step['kind'] == 'measure':
            if previous is not None and step['channel'] != previous:
                switches += 1
            previous = step['channel']

    groups = {name: {'stops': 0, 'moved_m': 0.0, 'measures': 0, 'clears': 0, 'seconds': 0.0}
              for name in ('search', 'track', 'clear', 'other')}
    for segment in _DIAG['segments']:
        bucket = groups[group_of(segment['kind'])]
        bucket['stops'] += 1
        bucket['moved_m'] += segment['moved_m']
        bucket['measures'] += segment['measures']
        bucket['clears'] += segment['clears']
    for name, bucket in groups.items():
        bucket['seconds'] = (bucket['moved_m'] / settings.ROBOT_SPEED_MPS
                             + bucket['measures'] * settings.MEASURE_TIME_S
                             + bucket['clears'] * (settings.OPTICAL_TIME_S + settings.CLEAR_TIME_S))
    total_moved = sum(b['moved_m'] for b in groups.values())

    virtual = float(transport.engine.virtual_time_s())
    cleared = len(client.state.cleared_channels)
    truth = scenario['jammer_count']
    directional = sum(1 for j in scenario['jammers'] if j.get('kind') == 'directional')
    excluded = sum(1 for c, info in (rec.get('channels') or {}).items()
                   if (info or {}).get('status') == 'excluded')
    stop_reason = rec.get('stop_reason')
    return {
        'seed': seed,
        'sources': truth,
        'directional_sources': directional,
        'cleared': cleared,
        'excluded_channels': excluded,
        'detected_uncleared': sum(1 for c, info in (rec.get('channels') or {}).items()
                                  if (info or {}).get('status') == 'detected'),
        'stop_reason': stop_reason,
        'complete': stop_reason in ('all_channels_resolved', 'cleared_limit') and cleared == truth,
        'virtual_time_s': virtual,
        'per_source_s': virtual / cleared if cleared else None,
        'moved_m': total_moved,
        'actions': len(steps),
        'measures': sum(1 for s in steps if s['kind'] == 'measure'),
        'clears': sum(1 for s in steps if s['kind'] == 'clear'),
        'switches': switches,
        'groups': groups,
        'search_stops': groups['search']['stops'],
        'search_measures': groups['search']['measures'],
        'segments': list(_DIAG['segments']),
        'track_stops': groups['track']['stops'],
        'track_measures': groups['track']['measures'],
        'clear_stops': groups['clear']['stops'],
    }


def aggregate(cases):
    ok = [c for c in cases if 'error' not in c]
    if not ok:
        return {}
    def mean(key):
        return sum(c[key] for c in ok) / len(ok)
    groups = {}
    for name in ('search', 'track', 'clear', 'other'):
        groups[name] = {
            'stops': sum(c['groups'][name]['stops'] for c in ok) / len(ok),
            'moved_m': sum(c['groups'][name]['moved_m'] for c in ok) / len(ok),
            'measures': sum(c['groups'][name]['measures'] for c in ok) / len(ok),
            'clears': sum(c['groups'][name]['clears'] for c in ok) / len(ok),
            'seconds': sum(c['groups'][name]['seconds'] for c in ok) / len(ok),
        }
    total = sum(groups[n]['seconds'] for n in groups)
    for name in groups:
        groups[name]['share'] = groups[name]['seconds'] / total if total else 0.0
    return {
        'cases': len(ok),
        'complete': sum(1 for c in ok if c['complete']),
        'mean_sources': mean('sources'),
        'mean_virtual_s': mean('virtual_time_s'),
        'mean_per_source_s': sum(c['per_source_s'] for c in ok if c['per_source_s']) / len(ok),
        'mean_moved_m': mean('moved_m'),
        'mean_measures': mean('measures'),
        'mean_clears': mean('clears'),
        'mean_switches': mean('switches'),
        'mean_search_stops': mean('search_stops'),
        'mean_search_measures': mean('search_measures'),
        'mean_track_stops': mean('track_stops'),
        'mean_track_measures': mean('track_measures'),
        'mean_clear_stops': mean('clear_stops'),
        'mean_excluded': mean('excluded_channels'),
        'groups': groups,
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--seeds', type=int, nargs='+', required=True)
    parser.add_argument('--overrides', default='{}')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    overrides = json.loads(args.overrides)
    _install_probe()
    cases = []
    for seed in args.seeds:
        case = run_one(seed, overrides)
        cases.append(case)
        if 'error' in case:
            print(json.dumps(case, ensure_ascii=False), flush=True)
            continue
        print(json.dumps({'seed': seed, 'src': case['sources'], 'dir': case['directional_sources'],
                          'ok': case['complete'], 'T': round(case['virtual_time_s'], 1),
                          'per_src': round(case['per_source_s'], 1) if case['per_source_s'] else None,
                          'move_m': round(case['moved_m']),
                          'meas': case['measures'], 'search_stop': case['search_stops'],
                          'search_meas': case['search_measures'],
                          'track_stop': case['track_stops'], 'clear_stop': case['clear_stops'],
                          'excl': case['excluded_channels'],
                          'move_s': round(case['groups']['search']['moved_m'] / 5),
                          'track_move_s': round(case['groups']['track']['moved_m'] / 5),
                          'clear_move_s': round(case['groups']['clear']['moved_m'] / 5)},
                         ensure_ascii=False), flush=True)
    payload = {'overrides': overrides, 'cases': cases, 'summary': aggregate(cases)}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
    print('=== SUMMARY ===', flush=True)
    print(json.dumps(payload['summary'], ensure_ascii=False, indent=2), flush=True)
    return payload


if __name__ == '__main__':
    main()
