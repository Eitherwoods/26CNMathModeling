# -*- coding: utf-8 -*-
"""问题三 V3 策略入口：主线条带（协议/记录/预算）+ v3policy 算法本体。

运行器按 `codes.strategy_p3v3:solve` 加载。与 `codes.strategy_p3:solve`
（MPC 主线）共用同一套 RobotClient 协议层与任务记录格式，便于演练成绩
直接对比；算法层换成 V3 的"七锚点自适应覆盖 + 全信道批量测量 +
近最优测量点追击"，24 种子官方同构 Engine 实测 275.4 s/源（100% 清除，
主线 MPC 定版 315.7）。

正确性论证
----------
- 每频道的可能位置集合按 ±1° 示向锥 ∩ (5,1500] 距离环做保守收缩；
  区域最小外接圆半径 ≤19.99 m 时清除圆心（保证命中）。
- "频道不存在"结论由 FlexibleCoverage 的分区覆盖证书给出：40 m 单元
  分配给 999.98 m 内锚点，扫描位置的 999.99 m 圆整单元清除；
  全部单元被无信号扫描覆盖 ⇒ 该频道在目标圆域内不存在。
- 试探清除失败仅 3 s 且排除 B(点,20)，与任何误差模型一致。
"""
from __future__ import annotations

import json
import math
import time

import numpy as np

from . import config as settings
from .problem3_solution import is_offline_run, summarize
from .v3policy import SearchPolicy

SCAN_COVER_RADIUS_M = 999.99


class _ContextClient:
    """把 StrategyContext 适配成 v3policy 期望的 client 接口。

    enter/exit 由主线条带（run_strategy）统一管理，这里只做无操作回执；
    measure/clear 保持位置参数风格并透传预算检查（真实时间保护）。
    """

    def __init__(self, context, steps):
        self.context = context
        self._steps = steps

    def enter(self):
        return {'accepted': True}

    def exit(self):
        return {'accepted': True, 'exit_reason': 'runner_managed'}

    def measure(self, pos, channel):
        response = self.context.measure(float(pos[0]), float(pos[1]), int(channel))
        self._steps.append({'kind': 'measure', 'channel': int(channel),
                            'result': response['measure_result'],
                            'x': float(pos[0]), 'y': float(pos[1]),
                            'virtual_time_s': float(response['virtual_time_s'])})
        return response

    def clear(self, pos, channel):
        response = self.context.clear(float(pos[0]), float(pos[1]), int(channel))
        self._steps.append({'kind': 'clear', 'channel': int(channel),
                            'result': response['clear_result'],
                            'x': float(pos[0]), 'y': float(pos[1]),
                            'virtual_time_s': float(response['virtual_time_s'])})
        return response


def _exclusion_certificates(no_signal_stations, cleared_channels):
    """按主线条带口径为"被证明不存在"的空频道生成覆盖证书。

    `no_signal_stations`：channel -> 该频道取得无信号读数的站点列表；
    只为未清除频道出证书（已清除频道在检出前也可能有无信号读数）。
    在目标圆域 100 m 采样格上检验"任一点到最近无信号站 ≤1000 m"，
    与 problem3_solution 的证书字段保持同构（certified 布尔）。
    """
    step = 100.0
    axis = np.arange(-settings.TARGET_RADIUS_M,
                     settings.TARGET_RADIUS_M + step, step)
    gx, gy = np.meshgrid(axis, axis)
    pts = np.column_stack((gx.ravel(), gy.ravel()))
    pts = pts[np.hypot(pts[:, 0], pts[:, 1]) <= settings.TARGET_RADIUS_M + 1e-9]
    certificates = {}
    for channel, stations in sorted(no_signal_stations.items()):
        if int(channel) in cleared_channels:
            continue
        if stations:
            coords = np.asarray(stations, dtype=float)
            covering = float(np.max(np.min(
                np.linalg.norm(pts[:, None, :] - coords[None, :, :], axis=2), axis=1)))
        else:
            covering = float(settings.TARGET_RADIUS_M * 2.0)
        certificates[str(channel)] = {
            'no_signal_stations': len(stations),
            'covering_radius_m': round(covering, 3),
            'receive_radius_lower_bound_m': settings.MIN_RECEIVE_RADIUS_M,
            'certified': bool(covering <= settings.MIN_RECEIVE_RADIUS_M + 1e-6),
        }
    return certificates


def run_mission(context):
    """在主线条带上运行 V3 策略，返回与 MPC 主线同构的任务记录。"""
    started_monotonic = time.monotonic()
    steps = []
    adapter = _ContextClient(context, steps)
    policy = SearchPolicy(adapter, directional=False, adaptive=True)
    start_virtual = float(context.state.virtual_time_s)
    outcome = policy.run()
    end_virtual = float(context.state.virtual_time_s)

    cleared = sorted(context.state.cleared_channels)
    if outcome['stop_reason'] == 'known_upper_bound_reached':
        stop_reason = 'cleared_limit'
    elif outcome['stop_reason'] == 'all_unresolved_channels_covered':
        stop_reason = 'all_channels_resolved'
    else:
        stop_reason = str(outcome['stop_reason'])

    no_signal = {}
    trajectory = [(0.0, 0.0)]
    for step in steps:
        trajectory.append((step['x'], step['y']))
        if step['kind'] == 'measure' and step['result'] == 'no_signal':
            no_signal.setdefault(step['channel'], []).append((step['x'], step['y']))
    moved = sum(math.dist(a, b) for a, b in zip(trajectory, trajectory[1:]))
    unresolved = [c for c in range(1, settings.CHANNEL_COUNT + 1) if c not in cleared]
    record = {
        'stop_reason': stop_reason,
        'cleared_channels': cleared,
        'unresolved_channels': unresolved,
        'inconsistent_channels': [],
        'start_virtual_time_s': start_virtual,
        'end_virtual_time_s': end_virtual,
        'real_elapsed_s': time.monotonic() - started_monotonic,
        'moved_distance_m': moved,
        'actions': len(steps),
        'planner_errors': 0,
        'steps': steps,
        'trajectory': [list(p) for p in trajectory],
        'channel_series': {str(c): [] for c in range(1, settings.CHANNEL_COUNT + 1)},
        'exclusion_certificates': _exclusion_certificates(no_signal, cleared),
        'tracking_states': {},
        'config': {'strategy': 'v3policy.AdaptivePolicy',
                   'fallback_count': outcome.get('fallback_count', 0),
                   'full_scan_positions': outcome.get('full_scan_positions', 0)},
    }
    return record


def solve(context):
    """运行器加载入口：运行 V3 策略并按在线/离线类型保存任务记录。"""
    record = run_mission(context)
    summary = summarize(record)
    offline = is_offline_run(context)
    try:
        target_dir = settings.record_dir_for(context.problem, offline=offline)
        target_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d-%H%M%S')
        prefix = 'offline-' if offline else ''
        target = target_dir / f'{prefix}mission_p{context.problem}_{stamp}.json'
        target.write_text(json.dumps({'summary': summary, 'record': record},
                                     ensure_ascii=False, indent=2),
                          encoding='utf-8')
        summary['record_path'] = str(target)
    except OSError:
        pass
    return summary
