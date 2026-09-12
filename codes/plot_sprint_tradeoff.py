# -*- coding: utf-8 -*-
"""问题三"清除率—平均清除时间"权衡折线图（速通前沿实验可视化）。

读取 `codes.sprint_frontier` 的输出 JSON，绘制 V3 策略在不同放弃证书阈值
K 下的（平均清除时间, 平均清除率）折线，并标注题目目标线（180 s/源）、
MPC 主线对照点与 100% 清除参考线。输出 PNG（预览）与 SVG（论文插入）。
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FIG_DIR = Path('figures/Problem3')


def plot(json_path: Path, fig_dir: Path = FIG_DIR):
    payload = json.loads(json_path.read_text(encoding='utf-8'))
    points = sorted(payload['points'], key=lambda p: p['mean_per_cleared_s'])
    xs = [p['mean_per_cleared_s'] for p in points]
    ys = [p['mean_clearance'] * 100.0 for p in points]

    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    fig, ax = plt.subplots(figsize=(7.2, 4.6), dpi=300)
    ax.plot(xs, ys, 'o-', color='#1f6fb5', linewidth=2.0, markersize=7,
            label='V3 速通前沿（K=10/12/14/16，24 种子）', zorder=5)
    for p in points:
        ax.annotate('K=%d\n%.1f s/源' % (p['min_clears'], p['mean_per_cleared_s']),
                    (p['mean_per_cleared_s'], p['mean_clearance'] * 100.0),
                    textcoords='offset points', xytext=(10, -16), fontsize=8,
                    color='#1f6fb5')
    # 100% 清除参考线与完整策略端点
    ax.axhline(100.0, color='#2a8a3e', linewidth=1.2, linestyle='--', alpha=0.8)
    ax.annotate('清除率 100%（完整覆盖证书，275.8 s/源）', (xs[-1], 100.0),
                textcoords='offset points', xytext=(-8, 8), ha='right',
                fontsize=8, color='#2a8a3e')
    # 题目目标线
    ax.axvline(180.0, color='#c0392b', linewidth=1.4, linestyle=':')
    ax.annotate('目标 180 s/源', (180.0, 83.0), textcoords='offset points',
                xytext=(6, 0), fontsize=8.5, color='#c0392b', rotation=90,
                va='bottom')
    # MPC 主线对照（速通与完整证书，12 种子口径）
    ax.scatter([235.0], [92.3], marker='s', s=46, facecolors='none',
               edgecolors='#d98200', linewidths=1.8, zorder=6,
               label='MPC 主线对照（速通 235.0 @ 92.3%；完整证书 315.7 @ 100%）')
    ax.scatter([315.7], [100.0], marker='s', s=46, facecolors='none',
               edgecolors='#d98200', linewidths=1.8, zorder=6)
    ax.annotate('速通', (235.0, 92.3), textcoords='offset points', xytext=(8, -4),
                fontsize=8, color='#d98200')
    ax.annotate('完整证书', (315.7, 100.0), textcoords='offset points',
                xytext=(-52, -14), fontsize=8, color='#d98200')

    ax.set_xlabel('平均清除时间（虚拟秒 / 实际清除源数，24 种子均值）')
    ax.set_ylabel('清除率（实际清除数 / 真实源数，%）')
    ax.set_title('问题三：放弃覆盖证书的速通前沿——清除率与平均清除时间的权衡')
    ax.set_xlim(170, 335)
    ax.set_ylim(76, 103)
    ax.grid(True, linewidth=0.4, alpha=0.5)
    ax.legend(loc='lower right', fontsize=8)
    fig.tight_layout()

    fig_dir.mkdir(parents=True, exist_ok=True)
    png = fig_dir / 'sprint_tradeoff.png'
    svg = fig_dir / 'sprint_tradeoff.svg'
    fig.savefig(png)
    fig.savefig(svg)
    plt.close(fig)
    print('saved ->', png, 'and', svg)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path,
                        default=Path('output/localsim/bench-zc/p3-sprint-frontier.json'))
    parser.add_argument('--fig-dir', type=Path, default=FIG_DIR)
    args = parser.parse_args(argv)
    plot(args.data, args.fig_dir)


if __name__ == '__main__':
    main()
