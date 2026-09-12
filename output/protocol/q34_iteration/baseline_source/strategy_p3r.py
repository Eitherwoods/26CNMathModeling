# -*- coding: utf-8 -*-
"""问题三 Route-First 策略入口（分支实验，strategy_mode='route'）。

运行器按 `codes.strategy_p3r:solve` 加载。与 `strategy_p3` 的唯一差别是
以 Route-First 模型（巡游 + 两站交会，见 problem3_route.py）求解。
"""
from .problem3_solution import Problem3Config
from .problem3_solution import run_mission, summarize, solve as _mpc_solve


def solve(context):
    """以 Route-First 模式运行并保存任务记录（记录逻辑与 MPC 入口一致）。"""
    import json
    import time
    from . import config as settings
    from .problem3_solution import is_offline_run

    record = run_mission(context, Problem3Config(strategy_mode='route'))
    summary = summarize(record)
    offline = is_offline_run(context)
    try:
        target_dir = settings.record_dir_for(context.problem, offline=offline)
        target_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d-%H%M%S')
        prefix = 'offline-' if offline else ''
        target = target_dir / f'{prefix}mission_p{context.problem}_{stamp}.json'
        target.write_text(json.dumps({'summary': summary, 'record': record},
                                     ensure_ascii=False, indent=2), encoding='utf-8')
        summary['record_path'] = str(target)
    except OSError as exc:
        summary['record_path'] = None
        summary['record_error'] = str(exc)
    return summary


__all__ = ['solve']
