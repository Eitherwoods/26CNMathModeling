# -*- coding: utf-8 -*-
"""V3 策略（strategy_p3v3 → v3policy.AdaptivePolicy）本地官方同构 Engine 基准。

与 benchmark_q34_local 不同，本脚本走 V3 路径（官方演练实际使用的策略），
单线程确定性，逐种子输出虚拟时间/清除数/停止原因，用于 A/B 对比环半径。
"""
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ.setdefault(_v, '1')
import json, sys, time, tempfile, statistics
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from codes.benchmark_q34_local import EngineTransport, generate_scenario
from codes.protocol import RobotClient
from codes.strategy import run_strategy
from codes.strategy_p3v3 import solve
from codes import config as settings


def run_seeds(seeds, tag):
    rows = []
    for seed in seeds:
        scenario = generate_scenario(3, seed)
        transport = EngineTransport(scenario)
        client = RobotClient('v3-bench', transport)
        t0 = time.perf_counter()
        err = None
        try:
            with tempfile.TemporaryDirectory() as tmp:
                with mock.patch.object(settings, 'PROBLEM3_OUTPUT_DIR', Path(tmp)):
                    result = run_strategy(client, lambda ctx: solve(ctx), problem=3)
        except Exception as exc:
            err = '%s: %s' % (type(exc).__name__, exc)
        wall = time.perf_counter() - t0
        truth = scenario['jammer_count']
        cleared = len(client.state.cleared_channels)
        row = dict(seed=seed, truth=truth, cleared=cleared,
                   virtual_s=transport.engine.virtual_time_s(), wall_s=wall, error=err)
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    ok = [r for r in rows if not r['error'] and r['cleared'] == r['truth']]
    out = dict(tag=tag, rows=rows,
               summary=dict(cases=len(rows), full_clear=len(ok),
                            mean_s_per_src=(statistics.mean(r['virtual_s'] / r['cleared'] for r in ok)
                                            if ok else None),
                            mean_virtual_s=(statistics.mean(r['virtual_s'] for r in ok) if ok else None)))
    return out


if __name__ == '__main__':
    seeds = list(range(1, int(sys.argv[1]) + 1)) if len(sys.argv) > 1 else list(range(1, 25))
    tag = sys.argv[2] if len(sys.argv) > 2 else 'run'
    out = run_seeds(seeds, tag)
    path = Path(__file__).parent / ('v3_%s.json' % tag)
    path.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')
    print('SUMMARY', json.dumps(out['summary'], ensure_ascii=False))
    print('saved', path)
