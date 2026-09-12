# -*- coding: utf-8 -*-
"""问题四本地测试案例工厂；不会建立连接，不代表官方案例分布。"""
from dataclasses import replace

import numpy as np

from .offline_stub import Source
from .scenario import Scenario, random_scenario


def mixed_scenario(seed=1, n_sources=12, n_directional=6):
    """生成指定数量的全向/定向混合源，仅将真值交给本地桩。"""
    if not 1 <= n_directional < n_sources <= 20:
        raise ValueError('混合案例要求两种源均非空，且总数不超过20。')
    base = random_scenario(seed=seed, n_sources=n_sources)
    rng = np.random.default_rng(seed)
    directions = rng.uniform(0, 360, n_directional)
    sources = tuple(replace(source, direction_deg=float(directions[i])) if i < n_directional else source
                    for i, source in enumerate(base.sources))
    return Scenario(f'mixed-seed{seed}-n{n_sources}-d{n_directional}', sources, seed,
                    '人工混合分布；性能不可解释为官方演练成绩。')


def boundary_scenario():
    """四个边界朝外定向源，加一个全向源，用于盲区回归而非正式规模。"""
    return Scenario('outward-boundary', (
        Source(1, 1800, 0, 1000, 0), Source(2, -1800, 0, 1000, 180),
        Source(3, 0, 1800, 1000, 90), Source(4, 0, -1800, 1000, 270),
        Source(5, 200, 100, 1000)), notes='5源缩小案例，仅检验边界几何。')
