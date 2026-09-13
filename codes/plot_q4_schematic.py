# -*- coding: utf-8 -*-
"""问题四模型示意图：(a) 定向源接收几何与三类反馈；(b) 同心环布站与发现保证。

数据与几何全部来自 `codes.config` 与 `codes.problem4_model.ring_search_waypoints`，
论文图 `figures/Problem4/q4_schematic.{png,svg}` 由本脚本生成。
"""
import math

import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Wedge

from .config import DPI, PROBLEM4_FIG_DIR, setup_matplotlib
from .problem4_model import ring_search_waypoints

STATION_COLOR = '#3775BA'
HIGHLIGHT_COLOR = '#E0A33E'
BEARING_ERROR_DEG = 1.0


def _plot_reception(ax):
    """定向源在检测点 X 处的可接收指示 v(X;h)：距离不超过 a 且位于发射半平面。"""
    bearing = 30.0
    receive_radius = 1200.0
    source = (0.0, 0.0)
    normal = (math.cos(math.radians(bearing)), math.sin(math.radians(bearing)))

    ax.add_patch(Wedge(source, receive_radius, bearing - 90.0, bearing + 90.0,
                       facecolor='#9EC5E8', alpha=0.35, edgecolor=STATION_COLOR,
                       linewidth=1.0, zorder=1))
    back = (-normal[0] * receive_radius, -normal[1] * receive_radius)
    ax.plot([source[0], back[0]], [source[1], back[1]], color='#9AA7B1',
            linestyle='--', linewidth=0.9, zorder=2)
    ax.annotate('发射半平面边界', (back[0] * 0.72, back[1] * 0.72),
                xytext=(-1750, -980), fontsize=8, color='#5B6B75',
                arrowprops=dict(arrowstyle='->', color='#5B6B75', lw=0.8))
    ax.annotate('', xy=(normal[0] * 1000.0, normal[1] * 1000.0), xytext=source,
                arrowprops=dict(arrowstyle='->', color='#B64342', lw=1.4))
    ax.annotate('$n(\\varphi)$', (normal[0] * 820, normal[1] * 820),
                xytext=(normal[0] * 940 + 60, normal[1] * 940), fontsize=10,
                color='#B64342')

    inside = (500.0, 400.0)
    ax.scatter(*inside, marker='s', s=46, color=STATION_COLOR, zorder=5)
    theta = math.degrees(math.atan2(source[1] - inside[1], source[0] - inside[0]))
    for delta in (-4.0, 4.0):
        edge = math.radians(theta + delta)
        ax.plot([inside[0], inside[0] + 620.0 * math.cos(edge)],
                [inside[1], inside[1] + 620.0 * math.sin(edge)],
                color='#B64342', linewidth=0.8, linestyle=':', zorder=3)
    ax.annotate('示向度 $\\theta\\pm1^\\circ$', inside, xytext=(760, 760),
                fontsize=8, arrowprops=dict(arrowstyle='->', lw=0.8))

    back_side = (-620.0, -320.0)
    far_side = (1700.0, 950.0)
    ax.scatter(*back_side, marker='o', s=40, color='#9AA7B1', zorder=5)
    ax.scatter(*far_side, marker='o', s=40, color='#9AA7B1', zorder=5)
    ax.annotate('无信号：位于背面', back_side, xytext=(-2150, -240), fontsize=8,
                arrowprops=dict(arrowstyle='->', lw=0.8))
    ax.annotate('无信号：距离过远', far_side, xytext=(1250, 1420), fontsize=8,
                arrowprops=dict(arrowstyle='->', lw=0.8))
    ax.scatter(*source, marker='*', s=170, color='#B64342', zorder=6)
    ax.annotate('定向源 $G,\\varphi$', source, xytext=(-560, -520), fontsize=9,
                arrowprops=dict(arrowstyle='->', lw=0.8))
    ax.set(xlim=(-2300, 2100), ylim=(-1250, 1550), aspect='equal',
           xlabel='x (m)', ylabel='y (m)', title='(a) 定向源联合假设的接收几何')
    ax.grid(alpha=0.2)


def _plot_layout(ax):
    """同心环 30 点布站与边界点 p 的 ±33.2° 楔形、1000 米覆盖圆。"""
    waypoints = np.asarray(ring_search_waypoints(), dtype=float)
    boundary = np.array([1800.0 * math.cos(math.radians(15.0)),
                         1800.0 * math.sin(math.radians(15.0))])
    distances = np.hypot(waypoints[:, 0] - boundary[0], waypoints[:, 1] - boundary[1])
    azimuth = np.degrees(np.arctan2(waypoints[:, 1], waypoints[:, 0]))
    delta_azimuth = (azimuth - 15.0 + 180.0) % 360.0 - 180.0
    covering = (distances <= 1000.0) & (np.abs(delta_azimuth) <= 33.2)

    ax.add_patch(Circle((0.0, 0.0), 1800.0, fill=False, color='#555555',
                        linestyle='--', linewidth=1.0, label='目标区域边界'))
    ax.scatter(waypoints[~covering, 0], waypoints[~covering, 1], marker='o',
               s=26, color=STATION_COLOR, zorder=4, label='扫描顶点（共30点）')
    ax.scatter(waypoints[covering, 0], waypoints[covering, 1], marker='D',
               s=46, color=HIGHLIGHT_COLOR, zorder=5, label='p 的覆盖顶点')
    ax.add_patch(Wedge((0.0, 0.0), 2380.0, 15.0 - 33.2, 15.0 + 33.2,
                       facecolor=HIGHLIGHT_COLOR, alpha=0.16,
                       edgecolor=HIGHLIGHT_COLOR, linewidth=0.8, zorder=1))
    ax.annotate('$\\pm33.2^\\circ$ 楔形', (2380.0 * math.cos(math.radians(48.0)),
                                      2380.0 * math.sin(math.radians(48.0))),
                xytext=(1450, 1560), fontsize=8, color='#9A6B1F',
                arrowprops=dict(arrowstyle='->', color='#9A6B1F', lw=0.8))
    ax.add_patch(Circle(boundary, 1000.0, fill=False, color='#2E8B57',
                        linestyle='--', linewidth=1.0, zorder=3))
    ax.plot([0.0, boundary[0]], [0.0, boundary[1]], color='#9AA7B1',
            linewidth=0.8, linestyle=':', zorder=2)
    ax.scatter(*boundary, marker='*', s=150, color='#272727', zorder=6)
    ax.annotate('边界点 $p$', boundary, xytext=(240, -420), fontsize=9,
                arrowprops=dict(arrowstyle='->', lw=0.8))
    ax.annotate('$1000$ 米覆盖圆', (boundary[0] - 707.0, boundary[1] + 707.0),
                xytext=(-1980, 1450), fontsize=8, color='#2E8B57',
                arrowprops=dict(arrowstyle='->', color='#2E8B57', lw=0.8))
    handles = [Line2D([], [], marker='o', linestyle='none', color=STATION_COLOR,
                      markersize=5),
               Line2D([], [], marker='D', linestyle='none', color=HIGHLIGHT_COLOR,
                      markersize=6)]
    ax.legend(handles, ['扫描顶点（共30点）', 'p 的覆盖顶点'], loc='lower left',
              fontsize=8)
    ax.set(xlim=(-2400, 2500), ylim=(-2400, 2500), aspect='equal',
           xlabel='x (m)', ylabel='y (m)', title='(b) 同心环布站与任意朝向发现保证')
    ax.grid(alpha=0.2)


def main(argv=None):
    plt = setup_matplotlib()
    fig, axes = plt.subplots(1, 2, figsize=(12.6, 5.4), layout='constrained')
    _plot_reception(axes[0])
    _plot_layout(axes[1])
    target = PROBLEM4_FIG_DIR / 'q4_schematic'
    for extension in ('png', 'svg'):
        fig.savefig(target.with_suffix(f'.{extension}'), dpi=DPI)
    plt.close(fig)
    print(target.with_suffix('.png'))
    return target.with_suffix('.png')


if __name__ == '__main__':
    main()
