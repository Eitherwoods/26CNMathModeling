# -*- coding: utf-8 -*-
"""A/B 对照：源数上限短路（upper_bound_shortcut）开与关，同种子同口径。

线程纪律（同 benchmark_q34_local）：多线程 BLAS 的归约顺序随负载/线程调度变化，
会被策略决策阈值放大成 ±3% 的过程级虚拟时间漂移，使 A/B 对比失效。本模块在
导入 numpy 前把线程数固定为 1，保证任意两次运行逐位可比；并强制单进程顺序执行，
每个变体内部先跑完再切下一个，避免两个变体交错放大漂移。

默认跑官方同构 Engine（可复现的确定性场景），`--engine offline` 才退回离线桩；
离线桩的噪声模型与官方不同，其数字**不可作为成绩**，只能看方向。
"""
from __future__ import annotations

import os
for _thread_var in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                    'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ.setdefault(_thread_var, '1')

import argparse
import json
import math
import os as _os
import statistics
import sys

sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))

# 官方 168 局 P3 实测源数分布（近似配比），用于按分布标准化。
OFFICIAL_MIX = {10: 9, 11: 10, 12: 11, 13: 5, 14: 5, 15: 9, 16: 7}


def _finite(value):
    """把非有限浮点（NaN/Inf）转成 None，保证 JSON 严格可解析。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def run_case_local(seed, config_overrides):
    """官方同构 Engine 单例运行，返回与离线路径一致的行结构。"""
    from codes.benchmark_q34_local import run_case
    out = run_case(3, seed, config_overrides)
    rec = out['record'] or {}
    return {
        'seed': seed, 'n': out['true_total'], 'cleared': out['cleared'],
        'true_total': out['true_total'],
        'complete': out['cleared'] == out['true_total'],
        'per_source_s': out['virtual_time_s'] / max(1, out['cleared']),
        'total_s': out['virtual_time_s'], 'moved_m': _finite(rec.get('moved_distance_m')),
        'actions': len(rec.get('steps') or []),
        'stop_reason': rec.get('stop_reason'),
        'constraints_ok': bool((out['checks'] or {}).get('all_constraints_ok')),
    }


def run_case_offline(seed, n_sources, config_overrides):
    """离线桩单例运行（数字不可作为成绩，仅看方向）。"""
    from codes.offline_stub import OfflineStub
    from codes.problem3_solution import Problem3Config, run_mission
    from codes.protocol import RobotClient
    from codes.scenario import random_scenario
    from codes.strategy import run_strategy

    sc = random_scenario(seed=seed, n_sources=n_sources)
    stub = OfflineStub(sources=sc.sources)
    client = RobotClient('offline-team', stub)
    cfg = Problem3Config(**config_overrides)
    rec = run_strategy(client, lambda ctx: run_mission(ctx, cfg), problem=3)['algorithm_result']
    total = rec['end_virtual_time_s'] - rec['start_virtual_time_s']
    cleared = len(rec['cleared_channels'])
    return {
        'seed': seed, 'n': sc.true_total, 'cleared': cleared, 'true_total': sc.true_total,
        'complete': cleared == sc.true_total,
        'per_source_s': total / max(1, cleared), 'total_s': total,
        'moved_m': _finite(rec.get('moved_distance_m')), 'actions': len(rec.get('steps') or []),
        'stop_reason': rec.get('stop_reason'), 'constraints_ok': True,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--engine', choices=('local', 'offline'), default='local',
                    help='local=官方同构 Engine（默认，可作成绩口径）；offline=离线桩')
    ap.add_argument('--seeds', type=int, default=8, help='离线模式下每个源数的种子数')
    ap.add_argument('--seed-list', type=int, nargs='+', default=None,
                    help='local 模式下的显式种子列表（建议复用 48 种子口径）')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    variants = {'off': {'upper_bound_shortcut': False},
                'on': {'upper_bound_shortcut': True}}
    results = {}

    if args.engine == 'local':
        seeds = args.seed_list or list(range(1, 25))
        for name, overrides in variants.items():
            rows = [run_case_local(seed, overrides) for seed in seeds]
            results[name] = rows
            print('%s: %d 局跑完' % (name, len(rows)), flush=True)
    else:
        cases = [(n, s) for n in sorted(OFFICIAL_MIX) for s in range(1, args.seeds + 1)]
        for name, overrides in variants.items():
            rows = [run_case_offline(s, n, overrides) for n, s in cases]
            results[name] = rows
            print('%s: %d 局跑完' % (name, len(rows)), flush=True)

    print()
    print('源数  局数   OFF每源    ON每源     变化     OFF行程   ON行程')
    overall_off, overall_on, per_n = [], [], {}
    for n in sorted({r['n'] for r in results['off']}):
        a = [r for r in results['off'] if r['n'] == n]
        b = [r for r in results['on'] if r['n'] == n]
        ma = statistics.mean(r['per_source_s'] for r in a)
        mb = statistics.mean(r['per_source_s'] for r in b)
        da = statistics.mean(r['moved_m'] or 0.0 for r in a)
        db = statistics.mean(r['moved_m'] or 0.0 for r in b)
        overall_off += [r['per_source_s'] for r in a]
        overall_on += [r['per_source_s'] for r in b]
        per_n[n] = (ma, mb, len(a))
        print('%3d  %4d  %7.1f  %7.1f  %+7.2f%%  %7.0f  %7.0f'
              % (n, len(a), ma, mb, (mb - ma) / ma * 100, da, db))

    print()
    print('算术均值:             OFF %.1f  ->  ON %.1f  (%+.2f%%)'
          % (statistics.mean(overall_off), statistics.mean(overall_on),
             (statistics.mean(overall_on) - statistics.mean(overall_off))
             / statistics.mean(overall_off) * 100))
    if set(per_n) <= set(OFFICIAL_MIX):
        weight = sum(OFFICIAL_MIX[n] for n in per_n)
        std_off = sum(OFFICIAL_MIX[n] * per_n[n][0] for n in per_n) / weight
        std_on = sum(OFFICIAL_MIX[n] * per_n[n][1] for n in per_n) / weight
        print('按官方源数分布标准化: OFF %.1f  ->  ON %.1f  (%+.2f%%)'
              % (std_off, std_on, (std_on - std_off) / std_off * 100))
    else:
        print('（源数集合超出官方配比，跳过分布标准化）')

    # 配对比较：逐种子比较，剔除样本方差的干扰。
    paired = []
    for a, b in zip(results['off'], results['on']):
        if a['seed'] == b['seed'] and a['per_source_s'] > 0:
            paired.append((b['per_source_s'] - a['per_source_s']) / a['per_source_s'] * 100)
    if paired:
        print('配对均值: %+.2f%%  中位 %+.2f%%  范围 %+.2f~%+.2f  (n=%d)'
              % (statistics.mean(paired), statistics.median(paired),
                 min(paired), max(paired), len(paired)))
    else:
        print('配对均值: 不适用')

    bad_off = sum(1 for r in results['off'] if not r['complete'])
    bad_on = sum(1 for r in results['on'] if not r['complete'])
    print('未完成局数: OFF %d / ON %d （必须为 0 0）' % (bad_off, bad_on))
    bad_cons = sum(1 for r in results['on'] if not r['constraints_ok'])
    print('ON 组约束未通过局数: %d （必须为 0）' % bad_cons)
    print('stop_reason(ON):', {k: sum(1 for r in results['on'] if r['stop_reason'] == k)
                               for k in sorted({r['stop_reason'] for r in results['on']})})

    if args.out:
        with open(args.out, 'w', encoding='utf-8') as fh:
            json.dump({'engine': args.engine, 'results': results}, fh,
                      ensure_ascii=False, indent=1)
        print('已写入', args.out)


if __name__ == '__main__':
    main()
