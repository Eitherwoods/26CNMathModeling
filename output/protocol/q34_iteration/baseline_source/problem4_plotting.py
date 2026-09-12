# -*- coding: utf-8 -*-
"""问题四任务图：轨迹与朝向、覆盖证据完成度、可能位置收敛、动作时间构成。

收敛图与时间构成两幅与问题三口径相同，直接复用 `problem3_plotting` 中的面板函数，
避免两处统计口径漂移；本模块只新增问题四特有的朝向与覆盖证据面板。
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Wedge

from .config import DPI, PROBLEM4_FIG_DIR, TARGET_RADIUS_M, setup_matplotlib
from .problem3_plotting import (CLEARED_COLOR, EXCLUDED_COLOR, PATH_COLOR, PENDING_COLOR,
                                _plot_convergence, _plot_timeline)

UNKNOWN_COLOR = '#7F7F7F'
ANOMALY_COLOR = '#E0A33E'
STATUS_COLORS = {'cleared': CLEARED_COLOR, 'excluded': EXCLUDED_COLOR,
                 'detected': PENDING_COLOR, 'unknown': UNKNOWN_COLOR,
                 'inconsistent': ANOMALY_COLOR}
STATUS_LABELS = {'cleared': '已清除', 'excluded': '已证明不存在', 'detected': '已发现未清除',
                 'unknown': '未证明', 'inconsistent': '模型异常'}
ORIENTATION_LENGTH_M = 400.0


def plot_mission(record, scenario=None, figures_dir=PROBLEM4_FIG_DIR, name=None):
    """绘制一次任务四联图：轨迹与朝向、覆盖证据、可能位置收敛、时间构成。"""
    plt = setup_matplotlib()
    figures_dir = Path(figures_dir)
    figures_dir.mkdir(parents=True, exist_ok=True)
    name = name or f"mission_{record.get('stop_reason', 'run')}"
    fig, axes = plt.subplots(1, 4, figsize=(22.0, 5.2), layout='constrained')

    _plot_trajectory(axes[0], record, scenario)
    _plot_evidence(axes[1], record)
    _plot_convergence(axes[2], record)
    _plot_timeline(axes[3], record)

    for extension in ('png', 'svg'):
        fig.savefig(figures_dir / f'{name}.{extension}', dpi=DPI)
    plt.close(fig)
    return figures_dir / f'{name}.png'


def _plot_trajectory(ax, record, scenario):
    """轨迹与干扰源；定向源另画发射法向与该朝向的正面接收半平面。"""
    ax.add_patch(Circle((0, 0), TARGET_RADIUS_M, fill=False, color='#555555',
                        linestyle='--', linewidth=0.8, label='目标区域边界'))
    trajectory = np.asarray(record['trajectory'], dtype=float)
    if len(trajectory) > 1:
        ax.plot(trajectory[:, 0], trajectory[:, 1], color=PATH_COLOR, linewidth=0.9,
                alpha=0.75, label='机器狗轨迹')
    ax.scatter(trajectory[0, 0], trajectory[0, 1], marker='s', s=40, color='#272727',
               zorder=6, label='起点')
    if scenario is not None:
        cleared = set(record['cleared_channels'])
        for source in scenario.sources:
            color = CLEARED_COLOR if source.channel in cleared else PENDING_COLOR
            ax.scatter(source.x, source.y, marker='*', s=100, color=color, zorder=7)
            ax.annotate(str(source.channel), (source.x, source.y), xytext=(4, 4),
                        textcoords='offset points', color=color, fontsize=8, zorder=8)
            if source.direction_deg is not None:
                ax.add_patch(Wedge((source.x, source.y), ORIENTATION_LENGTH_M,
                                   source.direction_deg - 90, source.direction_deg + 90,
                                   facecolor=color, alpha=0.13, edgecolor='none', zorder=3))
                head = np.array([source.x, source.y]) + ORIENTATION_LENGTH_M * np.array(
                    [np.cos(np.deg2rad(source.direction_deg)),
                     np.sin(np.deg2rad(source.direction_deg))])
                ax.annotate('', xy=head, xytext=(source.x, source.y), zorder=6,
                            arrowprops={'arrowstyle': '-|>', 'color': color, 'linewidth': 1.0})
        handles, labels = ax.get_legend_handles_labels()
        handles += [Line2D([], [], marker='*', linestyle='none', color=CLEARED_COLOR, markersize=9),
                    Line2D([], [], marker='*', linestyle='none', color=PENDING_COLOR, markersize=9),
                    Line2D([], [], color=PENDING_COLOR, linewidth=1.0, alpha=0.4)]
        ax.legend(handles, labels + ['已清除', '未清除', '定向源正面半平面'],
                  loc='best', fontsize=7.5)
    ax.set(xlabel='x (m)', ylabel='y (m)', aspect='equal',
           title=f"(a) 轨迹与目标（{len(record['cleared_channels'])} 个已清除）")
    ax.grid(alpha=0.2)


def _plot_evidence(ax, record):
    """逐频道覆盖证据完成度：只有一个频道测遍全部顶点才可作为不存在性证据。"""
    channels = sorted(record['channels'], key=lambda key: int(key))
    required = max([record['channels'][c]['coverage_required'] for c in channels] or [1])
    statuses = [record['channels'][c]['status'] for c in channels]
    progress = [record['channels'][c]['coverage_done'] / max(required, 1) for c in channels]
    ax.barh(np.arange(len(channels)), progress,
            color=[STATUS_COLORS.get(status, UNKNOWN_COLOR) for status in statuses], height=0.7)
    ax.axvline(1.0, color='#272727', linestyle='--', linewidth=0.9)
    ax.annotate(f'满覆盖 {required} 个顶点', (1.0, len(channels) - 0.4),
                xytext=(-4, 0), textcoords='offset points', ha='right', fontsize=8)
    ax.set(yticks=np.arange(len(channels)),
           yticklabels=[f'{c} {STATUS_LABELS.get(s, s)}' for c, s in zip(channels, statuses)],
           xlabel='已做无信号检测的搜索顶点比例', xlim=(0, 1.08),
           title=f"(b) 覆盖证据完成度（{record.get('search_waypoints', required)} 个搜索顶点）")
    ax.tick_params(axis='y', labelsize=7)
    ax.grid(alpha=0.2, axis='x')


def plot_mission_from_path(record_path, scenario=None, figures_dir=PROBLEM4_FIG_DIR, name=None):
    """从任务记录 JSON 绘制四联图。"""
    if not record_path:
        return None
    payload = json.loads(Path(record_path).read_text(encoding='utf-8'))
    return plot_mission(payload['record'], scenario=scenario, figures_dir=figures_dir, name=name)


def main(argv=None):
    """从已有任务记录出图；演练记录没有真值，因此不画干扰源与朝向。"""
    import argparse

    parser = argparse.ArgumentParser(description='从问题四任务记录 JSON 绘制四联图。')
    parser.add_argument('record', help='任务记录 JSON（离线自检或演练产生的均可）')
    parser.add_argument('--figures-dir', default=str(PROBLEM4_FIG_DIR))
    parser.add_argument('--name', default=None, help='输出文件名（不含扩展名）')
    args = parser.parse_args(argv)
    target = plot_mission_from_path(args.record, figures_dir=args.figures_dir, name=args.name)
    if target is None:
        raise SystemExit('没有可绘制的记录。')
    print(target)
    return target


if __name__ == '__main__':
    main()
