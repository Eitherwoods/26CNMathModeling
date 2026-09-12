# -*- coding: utf-8 -*-
"""问题三"速通前沿"实验：放弃覆盖证书收尾时，清除率与平均清除时间的权衡。

在本地官方同构 Engine（`codes.local_simulator`，1800 m 落点口径）上，把 V3
策略（`codes.v3policy`）的收尾条件从"全部未排除格点被覆盖证书锁死"替换为
"已清除 K 个源即停"（K=10/12/14，全部 20 个待清除时等价于完整策略），
扫描种子 1–24，记录每个 K 的平均清除时间（T/实际清除数）与平均清除率
（实际清除数/真实源数）。输出 JSON 供 `codes.plot_sprint_tradeoff` 作图。

这是"清除比例—平均定位清除时间"权衡的量化实验（题目两个统计量的张力），
只用于信息分析与论文讨论；正式演练策略保持完整证书（`codes.strategy_p3v3`）。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from .benchmark_q34_local import EngineTransport, generate_scenario
from .protocol import RobotClient
from .v3policy.adaptive_search_v3 import AdaptivePolicy
from .v3policy.efficient_search_v2 import target_route
from .v3policy.geometry import minimum_enclosing_circle


class SprintPolicy(AdaptivePolicy):
    """V3 策略的速通变体：cleared >= sprint_min_clears 即停止（放弃证书收尾）。"""

    sprint_min_clears = 16

    def run(self):
        if self.client.enter().get('accepted') is not True:
            raise RuntimeError('entry refused')
        while True:
            if len(self.cleared) >= self.sprint_min_clears:
                break
            choices = self.coverage.choices(self.position)
            if not choices:
                break
            owner, waypoint = min(choices, key=lambda pair: math.dist(self.position, pair[1]))
            self.scan_positions.append(waypoint)
            unresolved = [c for c in range(1, 21) if c not in self.cleared]
            if self.channel in unresolved:
                unresolved.remove(self.channel)
                unresolved.insert(0, self.channel)
            detected = []
            for channel in unresolved:
                result = self.measure(waypoint, channel)
                if result['measure_result'] != 'no_signal':
                    self.update(channel, waypoint, result)
                    detected.append((channel, result, waypoint))
            previous = len(self.coverage.remaining)
            self.coverage.record(waypoint, owner)
            if len(self.coverage.remaining) >= previous:
                raise RuntimeError('No certified search progress')
            while detected:
                centers = [minimum_enclosing_circle(self.regions[e[0]])[0]
                           if e[0] in self.regions else e[2] for e in detected]
                route = target_route(self.position, centers)
                entry = detected.pop(centers.index(route[0]))
                self.localize(*entry)
                for index, (channel, previous_result, previous_pos) in enumerate(detected):
                    result = self.measure(self.position, channel)
                    if result['measure_result'] != 'no_signal':
                        self.update(channel, self.position, result)
                        detected[index] = (channel, result, self.position)
            if len(self.cleared) == 16:
                self.stop_reason = 'known_upper_bound_reached'
                break
        if self.stop_reason is None:
            self.stop_reason = 'sprint_stop'
        self.client.exit()
        return dict(cleared=len(self.cleared), stop_reason=self.stop_reason)


class _AdapterClient:
    """v3policy 期望的 measure(pos, channel)/clear(pos, channel) 位置参数风格。"""

    def __init__(self, client):
        self.client = client

    def enter(self):
        return self.client.enter()

    def exit(self):
        return self.client.exit()

    def measure(self, pos, channel):
        return self.client.measure(pos[0], pos[1], channel)

    def clear(self, pos, channel):
        return self.client.clear(pos[0], pos[1], channel)


def run_point(problem, seeds, min_clears):
    """跑一个 K 档，返回逐种子记录与汇总。"""
    cases = []
    for seed in seeds:
        scenario = generate_scenario(problem, seed)
        transport = EngineTransport(scenario)
        client = RobotClient('sprint-frontier', transport)
        policy = SprintPolicy(_AdapterClient(client), directional=False, adaptive=True)
        policy.sprint_min_clears = min_clears
        policy.run()
        cleared = len(client.state.cleared_channels)
        truth = scenario['jammer_count']
        cases.append({'seed': seed, 'true_total': truth, 'cleared': cleared,
                      'virtual_time_s': transport.engine.virtual_time_s(),
                      'per_cleared_s': transport.engine.virtual_time_s() / max(cleared, 1),
                      'clearance': cleared / truth})
    mean_per = sum(c['per_cleared_s'] for c in cases) / len(cases)
    mean_clearance = sum(c['clearance'] for c in cases) / len(cases)
    return {'min_clears': min_clears, 'mean_per_cleared_s': round(mean_per, 3),
            'mean_clearance': round(mean_clearance, 4),
            'full_runs': sum(1 for c in cases if c['clearance'] >= 0.999),
            'worst_clearance': round(min(c['clearance'] for c in cases), 4),
            'cases': cases}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--problem', type=int, choices=(3,), default=3)
    parser.add_argument('--seeds', type=int, nargs='+', default=list(range(1, 25)))
    parser.add_argument('--min-clears', type=int, nargs='+', default=(10, 12, 14, 16))
    parser.add_argument('--output', type=Path,
                        default=Path('output/localsim/bench-zc/p3-sprint-frontier.json'))
    args = parser.parse_args(argv)
    points = [run_point(args.problem, args.seeds, k) for k in args.min_clears]
    payload = {'schema': 'p3-sprint-frontier-v1', 'problem': args.problem,
               'seeds': args.seeds, 'points': points}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
    for point in points:
        print('K=%2d per=%6.1f clearance=%.3f full=%2d/%d worst=%.2f' % (
            point['min_clears'], point['mean_per_cleared_s'], point['mean_clearance'],
            point['full_runs'], len(args.seeds), point['worst_clearance']))
    print('saved ->', args.output)


if __name__ == '__main__':
    main()
