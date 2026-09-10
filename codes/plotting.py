# -*- coding: utf-8 -*-
"""问题一几何证据图：测向布置、局部直径、覆盖反例。"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from .config import DPI, ANGLE_ERROR_DEG


def plot_region(observations, result, truth, figures_dir, name, counterexample=False):
    """保存布局与局部区域两面板，反例中额外显示直径圆。"""
    plt.rcParams.update({'font.sans-serif': ['Microsoft YaHei', 'SimHei', 'DejaVu Sans'],
                         'axes.unicode_minus': False, 'svg.fonttype': 'none',
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'legend.frameon': False, 'font.size': 9})
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8), layout='constrained')
    points = result.vertices
    for ax in axes:
        ax.fill(points[:, 0], points[:, 1], color='#3775BA', alpha=0.25, label='定位区域')
        ax.plot(*result.endpoints.T, color='#9A4D8E', linewidth=2, label='最远顶点对')
        ax.scatter(*truth, marker='*', color='#272727', s=90, label='设定真实位置', zorder=5)
        ax.set(xlabel='x (m)', ylabel='y (m)', aspect='equal')
    for i, (x, y, angle) in enumerate(observations):
        axes[0].scatter(x, y, color='#0F4D92', s=20)
        axes[0].annotate(f'S{i + 1}', (x, y), xytext=(4, 4), textcoords='offset points')
        length = np.linalg.norm(np.asarray([x, y]) - truth) * 1.12
        for delta in (-ANGLE_ERROR_DEG, 0, ANGLE_ERROR_DEG):
            direction = np.array([np.cos(np.deg2rad(angle + delta)), np.sin(np.deg2rad(angle + delta))])
            end = np.array([x, y]) + length * direction
            axes[0].plot([x, end[0]], [y, end[1]], color='#767676',
                         linestyle='-' if delta == 0 else '--', linewidth=0.6)
    if counterexample:
        axes[1].add_patch(Circle(result.endpoints.mean(axis=0), result.diameter / 2,
                                fill=False, color='#B64342', label='直径圆'))
    axes[0].set_title('(a) 人工验证场景与示向边界')
    axes[1].set_title(f'(b) 局部定位区域：D = {result.diameter:.6f} m')
    axes[1].margins(0.25)
    axes[1].legend(loc='best', fontsize=8)
    figures_dir.mkdir(parents=True, exist_ok=True)
    for extension in ('png', 'svg'):
        fig.savefig(figures_dir / f'{name}.{extension}', dpi=DPI)
    plt.close(fig)
