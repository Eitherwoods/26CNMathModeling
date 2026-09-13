# -*- coding: utf-8 -*-
""""放弃覆盖证书"权衡前沿图（问题三 + 问题四，论文讨论用）。

读取 `codes.sprint_frontier`（问题三）与 `codes.p4_sprint_frontier`（问题四）
的输出 JSON，绘制同一实验口径下的"平均定位清除时间—清除率"速通前沿：
K 越小（越早放弃证书收尾）时间越短、清除率越低；K=16 与默认完整策略等价。
每个面板叠加逐种子散点以展示分布；均值点标注 K 档位。

输出 SVG（论文插入，文字可编辑）+ PNG（预览），遵循 `可视化规范.md`。
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PALETTE = {
    'blue_main': '#0F4D92',
    'neutral_light': '#CFCECE',
    'neutral_mid': '#767676',
    'neutral_dark': '#4D4D4D',
}

PANELS = [
    ('a', '问题三：V3 策略（全向源，7 站覆盖证书）',
     Path('output/localsim/bench-zc/p3-sprint-frontier.json'),
     # 标签锚点（数据坐标，ha）：{K: (x, y, ha)}，细引线连到均值点
     {10: (196, 77.0, 'center'), 12: (221, 88.2, 'center'),
      14: (243, 95.6, 'center'), 16: (291, 97.2, 'center')}),
    ('b', '问题四：30 点方向位图证书策略',
     Path('output/localsim/bench-zc/p4-sprint-frontier.json'),
     {10: (516, 95.0, 'center'), 12: (527, 96.3, 'center'),
      14: (543, 97.6, 'center'), 16: (573, 99.6, 'left')}),
]


def setup_style():
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['svg.fonttype'] = 'none'
    plt.rcParams['axes.spines.right'] = False
    plt.rcParams['axes.spines.top'] = False
    plt.rcParams['axes.linewidth'] = 0.8
    plt.rcParams['xtick.major.width'] = 0.8
    plt.rcParams['ytick.major.width'] = 0.8
    plt.rcParams['legend.frameon'] = False
    plt.rcParams['font.size'] = 7
    plt.rcParams['axes.labelsize'] = 8
    plt.rcParams['xtick.labelsize'] = 6.5
    plt.rcParams['ytick.labelsize'] = 6.5
    plt.rcParams['legend.fontsize'] = 6.5
    plt.rcParams['lines.linewidth'] = 1.5


def draw_panel(ax, tag, title, payload, label_layout, n_seeds):
    points = sorted(payload['points'], key=lambda p: p['min_clears'])
    for point in points:  # 逐种子散点：展示每档 K 内的分布
        xs = [c['per_cleared_s'] for c in point['cases']]
        ys = [c['clearance'] * 100.0 for c in point['cases']]
        ax.scatter(xs, ys, s=9, color=PALETTE['neutral_light'], alpha=0.65,
                   linewidths=0, zorder=2,
                   label='逐种子结果' if point['min_clears'] == points[0]['min_clears'] else None)
    mean_x = [p['mean_per_cleared_s'] for p in points]
    mean_y = [p['mean_clearance'] * 100.0 for p in points]
    ax.plot(mean_x, mean_y, '-o', color=PALETTE['blue_main'], linewidth=1.5,
            markersize=4.5, zorder=4, label='K 档均值（速通前沿）')
    # 完整证书端点（K=16，与默认策略等价）单独强调
    ax.scatter([mean_x[-1]], [mean_y[-1]], s=64, marker='D',
               color=PALETTE['blue_main'], edgecolors='white', linewidths=0.8,
               zorder=5, label='完整证书（K=16，默认策略）')
    by_k = {p['min_clears']: p for p in points}
    for k, (lx, ly, ha) in label_layout.items():
        p = by_k[k]
        if k == points[-1]['min_clears']:
            label = '完整证书 K=16：%.1f s/源' % p['mean_per_cleared_s']
        else:
            label = 'K=%d：%.1f s/源' % (k, p['mean_per_cleared_s'])
        ax.annotate(label, (p['mean_per_cleared_s'], p['mean_clearance'] * 100.0),
                    xytext=(lx, ly), textcoords='data', ha=ha, va='center',
                    fontsize=6.5, color=PALETTE['blue_main'], zorder=6,
                    arrowprops=dict(arrowstyle='-', color=PALETTE['neutral_mid'],
                                    linewidth=0.5, shrinkA=1, shrinkB=3))
    ax.axhline(100.0, color=PALETTE['neutral_mid'], linewidth=1.0, linestyle='--',
               alpha=0.8, zorder=1)
    ax.annotate('清除率 100%', (0.985, 100.6), xycoords=('axes fraction', 'data'),
                ha='right', va='bottom', fontsize=6.5, color=PALETTE['neutral_mid'])
    ax.set_title(title, fontsize=8)
    ax.set_xlabel('平均定位清除时间（虚拟秒 / 被清除源数）')
    ax.set_ylabel('清除率（实际清除数 / 真实源数，%）')
    ax.text(0.985, 0.03, 'n = %d 种子/档' % n_seeds, transform=ax.transAxes,
            ha='right', va='bottom', fontsize=6.5, color=PALETTE['neutral_mid'])
    ax.text(-0.08, 1.12, tag, transform=ax.transAxes, fontsize=22,
            fontweight='bold', va='top', ha='right')
    ax.legend(loc='lower left', fontsize=6.5, handletextpad=0.4)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fig-dir', type=Path, default=Path('figures/Problem4'))
    parser.add_argument('--name', default='certificate_tradeoff')
    args = parser.parse_args(argv)
    setup_style()
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, (tag, title, path, layout) in zip(axes, PANELS):
        payload = json.loads(path.read_text(encoding='utf-8'))
        draw_panel(ax, tag, title, payload, layout, len(payload['seeds']))
    fig.tight_layout()
    args.fig_dir.mkdir(parents=True, exist_ok=True)
    svg = args.fig_dir / f'{args.name}.svg'
    png = args.fig_dir / f'{args.name}.png'
    fig.savefig(svg, bbox_inches='tight')
    fig.savefig(png, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print('saved ->', svg, 'and', png)


if __name__ == '__main__':
    main()
