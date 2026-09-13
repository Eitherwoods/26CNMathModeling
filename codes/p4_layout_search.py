# -*- coding: utf-8 -*-
"""问题四搜索布站优化：以"虚拟时间"为目标，以方向覆盖证书为硬约束。

成本模型（每局，平均空频道数 E）
--------------------------------
搜索阶段的虚拟时间 =

    tour_m / 5           巡回移动
  + N * E * 5            每个顶点对每个未知频道一次检测
  + N * E * 1            频道切换（按频道号顺序扫描时每次切换 1 s）

所以"删一个顶点"的收益 = E * 6 s（E≈7 时 42 s，相当于省 210 m 巡回），
"缩短巡回"的收益 = 每 5 m 换 1 s。两者必须联合优化，不能只看点数或只看长度。

约束：所有位置单元 × 全部 24 个朝向 bin 都必须被站点集合证伪，
判据用求解器自身的 `search_evidence.DirectionalCoverage`（位图，严格），
不用偏松的连续角空隙近似。

用法：
    python -m codes.p4_layout_search --empty 7 --rounds 3
"""
from __future__ import annotations
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ.setdefault(_v, '1')

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .base_models import open_route_order
from .config import ROBOT_SPEED_MPS, MEASURE_TIME_S, CHANNEL_SWITCH_TIME_S
from .problem3_model import Lattice
from .problem4_model import ring_search_waypoints, optimized_search_waypoints
from .search_evidence import DirectionalCoverage

MAX_STATION_RADIUS_M = 2700.0


class LayoutEvaluator:
    """证书完整性 + 巡回长度的联合评估器（带缓存）。"""

    def __init__(self, lattice_spacing_m=20.0, orientation_bins=24, tour_starts=8):
        self.lattice = Lattice.build(lattice_spacing_m)
        self.coverage = DirectionalCoverage(self.lattice, orientation_bins)
        self.tour_starts = tour_starts
        self._complete = {}
        self._tour = {}
        self.evaluations = 0

    def _key(self, points):
        return tuple(sorted((round(float(x), 6), round(float(y), 6)) for x, y in points))

    def complete(self, points):
        key = self._key(points)
        hit = self._complete.get(key)
        if hit is None:
            mask = self.coverage.new_mask()
            for point in points:
                mask &= self.coverage.station_mask(np.asarray(point, dtype=float))
                if not mask.any():
                    break
            hit = not bool(mask.any())
            self._complete[key] = hit
        return hit

    def tour_m(self, points, start=(0.0, 0.0)):
        key = (self._key(points), round(float(start[0]), 6), round(float(start[1]), 6))
        hit = self._tour.get(key)
        if hit is None:
            array = np.asarray(points, dtype=float)
            if len(array) == 0:
                hit = 0.0
            else:
                order = open_route_order(array, np.asarray(start, dtype=float),
                                         starts=self.tour_starts)
                path = np.vstack((np.asarray(start, dtype=float), array[order]))
                hit = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
            self._tour[key] = hit
        return hit

    def cost_s(self, points, empty_channels, start=(0.0, 0.0)):
        """搜索阶段虚拟时间；证书不完整的布站返回无穷大。"""
        self.evaluations += 1
        if not self.complete(points):
            return float('inf')
        n = len(points)
        return (self.tour_m(points, start) / ROBOT_SPEED_MPS
                + n * empty_channels * (MEASURE_TIME_S + CHANNEL_SWITCH_TIME_S))

    def residual(self, points):
        """证书残留的 (单元, 朝向) 位数；0 表示完整。"""
        mask = self.coverage.new_mask()
        for point in points:
            mask &= self.coverage.station_mask(np.asarray(point, dtype=float))
        return int(np.unpackbits(mask.view(np.uint8)).sum())


def polar_candidates(radii, angle_step_deg=6.0, max_radius=MAX_STATION_RADIUS_M):
    """极坐标候选站点集（含原点）。"""
    points = [(0.0, 0.0)]
    for radius in radii:
        if radius <= 0 or radius > max_radius:
            continue
        count = max(1, int(round(360.0 / angle_step_deg)))
        for i in range(count):
            angle = math.radians(i * 360.0 / count)
            points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def greedy_prune(evaluator, points, empty_channels, verbose=True):
    """逐个删除使成本下降最多的顶点，直到无法再删。"""
    current = [tuple(p) for p in points]
    best_cost = evaluator.cost_s(current, empty_channels)
    if verbose:
        print(f'[prune] start n={len(current)} tour={evaluator.tour_m(current):.0f} '
              f'cost={best_cost:.1f}', flush=True)
    improved = True
    while improved:
        improved = False
        scored = []
        for index in range(len(current)):
            trial = current[:index] + current[index + 1:]
            cost = evaluator.cost_s(trial, empty_channels)
            if cost < best_cost - 1e-9:
                scored.append((cost, index))
        if scored:
            scored.sort()
            best_cost, index = scored[0]
            current = current[:index] + current[index + 1:]
            improved = True
            if verbose:
                print(f'[prune] n={len(current)} tour={evaluator.tour_m(current):.0f} '
                      f'cost={best_cost:.1f}', flush=True)
    return current, best_cost


def local_search(evaluator, points, empty_channels, radii, angle_step_deg,
                 rounds=2, verbose=True):
    """对每个顶点做半径/角度扰动，接受成本下降的替换。"""
    current = [tuple(p) for p in points]
    best_cost = evaluator.cost_s(current, empty_channels)
    radius_choices = sorted(set(radii) | {0.0})
    for round_index in range(rounds):
        improved = False
        for index in range(len(current)):
            x, y = current[index]
            radius = math.hypot(x, y)
            angle = math.atan2(y, x)
            trials = []
            for step_r in (-200.0, -100.0, -50.0, 50.0, 100.0, 200.0):
                trials.append((radius + step_r, angle))
            for step_a in (-12.0, -6.0, -3.0, 3.0, 6.0, 12.0):
                trials.append((radius, angle + math.radians(step_a)))
            best_trial, best_trial_cost = None, best_cost
            for new_radius, new_angle in trials:
                if new_radius <= 1e-9 or new_radius > MAX_STATION_RADIUS_M:
                    continue
                point = (new_radius * math.cos(new_angle), new_radius * math.sin(new_angle))
                trial = list(current)
                trial[index] = point
                cost = evaluator.cost_s(trial, empty_channels)
                if cost < best_trial_cost - 1e-9:
                    best_trial, best_trial_cost = point, cost
            if best_trial is not None:
                current[index] = best_trial
                best_cost = best_trial_cost
                improved = True
        if verbose:
            print(f'[local] round {round_index} n={len(current)} '
                  f'tour={evaluator.tour_m(current):.0f} cost={best_cost:.1f}', flush=True)
        if not improved:
            break
    return current, best_cost


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--empty', type=float, default=7.0,
                        help='平均空频道数（20 − 期望源数），决定每顶点的检测成本')
    parser.add_argument('--rounds', type=int, default=2)
    parser.add_argument('--seed-layout', default='rings', choices=('rings', 'optimized'))
    parser.add_argument('--angle-step', type=float, default=6.0)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)

    evaluator = LayoutEvaluator()
    radii = tuple(np.arange(200.0, MAX_STATION_RADIUS_M + 1e-9, 100.0))
    print(f'lattice points = {len(evaluator.lattice.points)}, '
          f'cover radius = {evaluator.lattice.covering_radius_m:.3f}', flush=True)

    start_points = (ring_search_waypoints() if args.seed_layout == 'rings'
                    else optimized_search_waypoints())
    baseline = [tuple(p) for p in start_points]
    base_cost = evaluator.cost_s(baseline, args.empty)
    print(f'[base ] {args.seed_layout} n={len(baseline)} '
          f'tour={evaluator.tour_m(baseline):.0f} residual={evaluator.residual(baseline)} '
          f'cost={base_cost:.1f}', flush=True)

    pruned, cost = greedy_prune(evaluator, baseline, args.empty)
    moved, cost = local_search(evaluator, pruned, args.empty, radii, args.angle_step,
                               rounds=args.rounds)
    pruned2, cost = greedy_prune(evaluator, moved, args.empty)
    print(f'[final] n={len(pruned2)} tour={evaluator.tour_m(pruned2):.0f} '
          f'residual={evaluator.residual(pruned2)} cost={cost:.1f} '
          f'(base {base_cost:.1f}, {100*(cost-base_cost)/base_cost:+.2f}%)', flush=True)
    print('evaluations =', evaluator.evaluations, flush=True)
    print('waypoints =', json.dumps([[round(x, 2), round(y, 2)] for x, y in pruned2]), flush=True)

    payload = {'empty_channels': args.empty, 'seed_layout': args.seed_layout,
               'baseline': {'n': len(baseline), 'tour_m': evaluator.tour_m(baseline),
                            'cost_s': base_cost, 'residual': evaluator.residual(baseline)},
               'optimized': {'n': len(pruned2), 'tour_m': evaluator.tour_m(pruned2),
                             'cost_s': cost, 'residual': evaluator.residual(pruned2),
                             'waypoints': [[round(x, 2), round(y, 2)] for x, y in pruned2]},
               'evaluations': evaluator.evaluations}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
    return payload


if __name__ == '__main__':
    main()
