# -*- coding: utf-8 -*-
"""问题三算法入口。

运行器按 `codes.strategy_p3:solve` 加载本模块，只暴露一个 `solve(context)`，
策略实现全部放在 `problem3_solution.py`，便于单独测试与替换。
"""
from .problem3_solution import run_mission, summarize, solve  # noqa: F401  对外接口

__all__ = ['solve', 'run_mission', 'summarize']
