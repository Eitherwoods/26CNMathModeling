# -*- coding: utf-8 -*-
"""问题三任务图：行进轨迹、可能区域收敛、时间构成。"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle

from .config import DPI, PROBLEM3_FIG_DIR, TARGET_RADIUS_M, setup_matplotlib

CLEARED_COLOR = '#B64342'
PENDING_COLOR = '#0F4D92'
EXCLUDED_COLOR = '#9AA7B1'
PATH_COLOR = '#3775BA'


def plot_mission(record, scenario=None, figures_dir=PROBLEM3_FIG_DIR, name=None):
    """绘制一次任务的三联图：轨迹、可能区域收敛、动作时间构成。"""
    plt = setup_matplotlib()
    figures_dir = Path(figures_dir)
    figures_dir.mkdir(parents=True, exist_ok=True)
    name = name or f"mission_{record.get('stop_reason', 'run')}"
    fig, axes = plt.subplots(1, 3, figsize=(16.5, 5.0), layout='constrained')

    _plot_trajectory(axes[0], record, scenario)
    _plot_convergence(axes[1], record)
    _plot_timeline(axes[2], record)

    for extension in ('png', 'svg'):
        fig.savefig(figures_dir / f'{name}.{extension}', dpi=DPI)
    plt.close(fig)
    return figures_dir / f'{name}.png'


def _plot_trajectory(ax, record, scenario):
    """轨迹与干扰源：已清除与未清除分别标注。"""
    ax.add_patch(Circle((0, 0), TARGET_RADIUS_M, fill=False, color='#555555',
                        linestyle='--', linewidth=0.8, label='目标区域边界'))
    trajectory = np.asarray(record['trajectory'], dtype=float)
    if len(trajectory) > 1:
        ax.plot(trajectory[:, 0], trajectory[:, 1], color=PATH_COLOR, linewidth=1.0,
                alpha=0.8, label='机器狗轨迹')
    ax.scatter(trajectory[0, 0], trajectory[0, 1], marker='s', s=40,
               color='#272727', zorder=6, label='起点')
    if scenario is not None:
        cleared = set(record['cleared_channels'])
        for source in scenario.sources:
            color = CLEARED_COLOR if source.channel in cleared else PENDING_COLOR
            ax.scatter(source.x, source.y, marker='*', s=90, color=color, zorder=5)
            ax.annotate(str(source.channel), (source.x, source.y), xytext=(4, 4),
                        textcoords='offset points', color=color, fontsize=8)
        handles, labels = ax.get_legend_handles_labels()
        handles += [Line2D([], [], marker='*', linestyle='none', color=CLEARED_COLOR, markersize=9),
                    Line2D([], [], marker='*', linestyle='none', color=PENDING_COLOR, markersize=9)]
        ax.legend(handles, labels + ['已清除', '未清除'], loc='best', fontsize=8)
    ax.set(xlabel='x (m)', ylabel='y (m)', aspect='equal',
           title=f"(a) 轨迹与目标（{len(record['cleared_channels'])} 个已清除）")
    ax.grid(alpha=0.2)


def _plot_convergence(ax, record, panel='(b)'):
    """各频道可能位置集合的最坏剩余距离随检测次数下降。"""
    drawn = 0
    for channel, series in sorted(record['channel_series'].items(), key=lambda item: int(item[0])):
        limits = [(point['measure_count'], point['limit_m']) for point in series
                  if point['limit_m'] is not None]
        if len(limits) < 2:
            continue
        counts, values = zip(*limits)
        ax.plot(counts, values, marker='o', markersize=2.5, linewidth=0.9, alpha=0.85,
                label=f'频道 {channel}')
        drawn += 1
    ax.axhline(20.0, color=CLEARED_COLOR, linestyle='--', linewidth=0.9)
    ax.annotate('清除半径 20 m', (0.98, 20.0), xycoords=('axes fraction', 'data'),
                ha='right', va='bottom', fontsize=8, color=CLEARED_COLOR)
    ax.set_yscale('log')
    ax.set(xlabel='该频道的检测次数', ylabel='可能位置到当前位置的最远距离 (m)',
           title=f'{panel} 可能位置集合收敛')
    if drawn:
        ax.legend(fontsize=7, ncol=2)
    ax.grid(alpha=0.2, which='both')


def _plot_timeline(ax, record, panel='(c)'):
    """动作耗时的构成：移动、检测、频道切换与清除。"""
    steps = record['steps']
    if not steps:
        ax.set_title(f'{panel} 动作时间构成（无动作）')
        return
    virtual = np.array([step['virtual_time_s'] for step in steps], dtype=float)
    deltas = np.diff(np.concatenate(([record['start_virtual_time_s']], virtual)))
    moved = np.array([0.0] + [np.hypot(steps[i]['x'] - steps[i - 1]['x'],
                                       steps[i]['y'] - steps[i - 1]['y'])
                              for i in range(1, len(steps))])
    travel = np.minimum(deltas, moved / 5.0)
    measure_index = np.array([step['kind'] == 'measure' for step in steps])
    clear_index = ~measure_index
    switching = np.array([1.0 if (measure_index[i] and i > 0 and measure_index[i - 1]
                                  and steps[i]['channel'] != steps[i - 1]['channel']) else 0.0
                          for i in range(len(steps))])
    detection = np.where(measure_index, np.maximum(deltas - travel - switching, 0.0), 0.0)
    clearing = np.where(clear_index, np.maximum(deltas - travel, 0.0), 0.0)
    ax.bar(1, float(travel.sum()), color=PATH_COLOR, label='移动')
    ax.bar(1, float(detection.sum()), bottom=float(travel.sum()), color='#9A4D8E',
           label='检测与调整天线')
    ax.bar(1, float(switching.sum()), bottom=float(travel.sum() + detection.sum()),
           color='#E0A33E', label='频道切换')
    ax.bar(1, float(clearing.sum()), bottom=float(travel.sum() + detection.sum() + switching.sum()),
           color=CLEARED_COLOR, label='光学定位与清除')
    total = float(deltas.sum())
    ax.set(xlabel='整局任务', ylabel='虚拟时间 (s)',
           title=f'{panel} 动作时间构成（合计 {total:.0f} s）', xticks=[])
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(alpha=0.2, axis='y')


def plot_mission_from_path(record_path, scenario=None, figures_dir=PROBLEM3_FIG_DIR):
    """从任务记录 JSON 绘制三联图。"""
    if not record_path:
        return None
    payload = json.loads(Path(record_path).read_text(encoding='utf-8'))
    return plot_mission(payload['record'], scenario=scenario, figures_dir=figures_dir)
