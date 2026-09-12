# -*- coding: utf-8 -*-
"""离线演练案例生成：随机全向干扰源案例与确定性手工案例。

生成器只用于本地桩演练与自检，不是官方模拟器的案例分布。
频道互不相同，位置在目标圆域内均匀取样，有效接收半径在 1000~1500 m 内。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import CHANNEL_COUNT, MAX_SOURCE_COUNT, MIN_SOURCE_COUNT, TARGET_RADIUS_M
from .offline_stub import Source


@dataclass(frozen=True)
class Scenario:
    """一次演练案例：干扰源集合与生成信息。"""

    name: str
    sources: tuple
    seed: int | None = None
    notes: str = ''

    @property
    def true_total(self):
        return len(self.sources)

    def channels(self):
        return sorted(source.channel for source in self.sources)


def random_scenario(seed=None, n_sources=None, radius_m=TARGET_RADIUS_M,
                    min_separation_m=80.0, location_error_deg=0.9,
                    radius_range_m=(1000.0, 1500.0), max_attempts=500):
    """生成一个随机案例：频道互不相同、位置近似均匀、半径落在有效区间内。"""
    rng = np.random.default_rng(seed)
    count = int(n_sources) if n_sources else int(rng.integers(MIN_SOURCE_COUNT, MAX_SOURCE_COUNT + 1))
    if not 1 <= count <= CHANNEL_COUNT:
        raise ValueError(f'案例干扰源个数必须在 1~{CHANNEL_COUNT} 之间。')
    channels = rng.choice(CHANNEL_COUNT, size=count, replace=False) + 1
    positions = []
    for _ in range(max_attempts * count):
        if len(positions) == count:
            break
        angle = rng.uniform(0.0, 2.0 * np.pi)
        distance = radius_m * np.sqrt(rng.uniform(0.0, 1.0))
        candidate = np.array([distance * np.cos(angle), distance * np.sin(angle)])
        if all(np.linalg.norm(candidate - other) >= min_separation_m for other in positions):
            positions.append(candidate)
    if len(positions) < count:
        raise RuntimeError('无法在给定最小间距下放置全部干扰源。')
    sources = tuple(
        Source(channel=int(channel), x=float(position[0]), y=float(position[1]),
               radius=float(rng.uniform(*radius_range_m)),
               bearing_error_deg=0.0)
        for channel, position in zip(channels, positions))
    return Scenario(name=f'random-seed{seed}-n{count}', sources=sources, seed=seed,
                    notes='随机案例，仅用于本地桩演练；官方案例分布未知。')


def fixed_scenario(points, name='fixed', radius_m=1200.0):
    """按 [(频道, x, y), ...] 生成确定性案例，用于单元测试。"""
    sources = tuple(Source(channel=int(channel), x=float(x), y=float(y), radius=float(radius_m))
                    for channel, x, y in points)
    return Scenario(name=name, sources=sources, seed=None, notes='确定性手工案例。')


def sweep_waypoints(ring_radius_m=1200.0, radius_m=TARGET_RADIUS_M):
    """中心加六边形外圈共七个覆盖停靠点（见 problem3_model.hex_waypoints）。"""
    from .problem3_model import hex_waypoints
    return hex_waypoints(ring_radius_m, radius_m)
