# -*- coding: utf-8 -*-
"""问题四"速通前沿"实验：放弃空频道排除证书时，清除率与平均清除时间的权衡。

在本地官方同构 Engine（`codes.local_simulator`，1800 m 落点口径）上，把
`codes.problem4_solution` 的结束条件放宽为"已清除 ≥ K 个源且已发现频道全部
清完即停"（不再继续搜索以证明其余频道不存在；K=16 时与默认 `cleared_limit`
短路等价，即完整策略对照），扫描种子 1–24，记录每个 K 的平均清除时间
（T/实际清除数）与平均清除率（实际清除数/真实源数）。输出 JSON 供
`codes.plot_certificate_tradeoff` 作图。

与 `codes.sprint_frontier`（问题三 V3 版）同构：这是题目两个统计量
"清除比例 × 平均定位清除时间"张力的量化实验，只用于信息分析与论文讨论；
正式演练策略保持完整证书（30 点方向位图覆盖证据）。

实现说明：只子类化 `Problem4Strategy` 重载 `finished()`，不修改正式策略；
`search_interval=1` 交错调度下，停止发生在"当前已发现频道全部清完"的轮次
边界，语义与问题三 SprintPolicy（清完已发现即停）一致。
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .benchmark_q34_local import EngineTransport, generate_scenario
from .protocol import RobotClient
from .strategy import run_strategy
from .problem4_solution import Problem4Config, Problem4Strategy


class SprintStrategy(Problem4Strategy):
    """P4 策略的速通变体：已发现频道全部清完且 cleared >= K 即停。"""

    sprint_min_clears = 16

    def finished(self):
        reason = super().finished()
        if reason is not None:
            return reason
        cleared = sum(k.status == 'cleared' for k in self.channels.values())
        detected = any(k.status == 'detected' for k in self.channels.values())
        if cleared >= self.sprint_min_clears and not detected:
            return 'sprint_stop'
        return None


def run_point(seeds, min_clears):
    """跑一个 K 档，返回逐种子记录与汇总。"""
    cases = []
    for seed in seeds:
        scenario = generate_scenario(4, seed)
        transport = EngineTransport(scenario)
        client = RobotClient('p4-sprint-frontier', transport)
        config = Problem4Config()

        def solver(ctx, limit=min_clears):
            strategy = SprintStrategy(ctx, config)
            strategy.sprint_min_clears = limit
            return strategy.run()

        result = run_strategy(client, solver, problem=4)
        cleared = len(client.state.cleared_channels)
        truth = scenario['jammer_count']
        record = result.get('algorithm_result') or {}
        steps = record.get('steps') or []
        clear_times = [s['virtual_time_s'] for s in steps if s['kind'] == 'clear']
        tail_s = (float(record.get('end_virtual_time_s', 0.0)) - max(clear_times)
                  if clear_times else None)
        cases.append({'seed': seed, 'true_total': truth, 'cleared': cleared,
                      'virtual_time_s': float(result['virtual_time_s']),
                      'per_cleared_s': float(result['virtual_time_s']) / max(cleared, 1),
                      'clearance': cleared / truth,
                      'stop_reason': record.get('stop_reason'),
                      'certificate_tail_s': tail_s})
    mean_per = sum(c['per_cleared_s'] for c in cases) / len(cases)
    mean_clearance = sum(c['clearance'] for c in cases) / len(cases)
    tails = [c['certificate_tail_s'] for c in cases if c['certificate_tail_s'] is not None]
    return {'min_clears': min_clears, 'mean_per_cleared_s': round(mean_per, 3),
            'mean_clearance': round(mean_clearance, 4),
            'full_runs': sum(1 for c in cases if c['clearance'] >= 0.999),
            'worst_clearance': round(min(c['clearance'] for c in cases), 4),
            'mean_certificate_tail_s': round(sum(tails) / len(tails), 1) if tails else None,
            'cases': cases}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seeds', type=int, nargs='+', default=list(range(1, 25)))
    parser.add_argument('--min-clears', type=int, nargs='+', default=(10, 12, 14, 16))
    parser.add_argument('--output', type=Path,
                        default=Path('output/localsim/bench-zc/p4-sprint-frontier.json'))
    args = parser.parse_args(argv)
    points = [run_point(args.seeds, k) for k in args.min_clears]
    payload = {'schema': 'p4-sprint-frontier-v1', 'problem': 4,
               'seeds': args.seeds, 'points': points}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
    for point in points:
        print('K=%2d per=%6.1f clearance=%.3f full=%2d/%d worst=%.2f tail=%s' % (
            point['min_clears'], point['mean_per_cleared_s'], point['mean_clearance'],
            point['full_runs'], len(args.seeds), point['worst_clearance'],
            point['mean_certificate_tail_s']))
    print('saved ->', args.output)


if __name__ == '__main__':
    main()
