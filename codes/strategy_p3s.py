# -*- coding: utf-8 -*-
"""问题三算法入口（速通收尾口径）。

与 `strategy_p3` 唯一区别：`Problem3Config(sprint_stop=True)`——清除数达到
`sprint_min_clears` 且无已发现待追踪频道时，放弃"证明剩余频道不存在"的
证书收尾，以 'sprint_stop' 提前结束。这是"清除比例 vs 平均定位清除时间"
的显式权衡口径，供演练对照与论文消融使用；保守口径仍以
`codes.strategy_p3:solve` 为准。
"""
from .problem3_solution import Problem3Config, run_mission, summarize  # noqa: F401
from .problem3_solution import solve as _solve_with_config  # noqa: F401


def solve(context):
    """速通口径的算法入口：记录格式与保守口径完全一致。"""
    return _solve_with_config(context, Problem3Config(sprint_stop=True))


__all__ = ['solve', 'run_mission', 'summarize']
