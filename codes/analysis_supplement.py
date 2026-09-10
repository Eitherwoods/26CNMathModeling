# -*- coding: utf-8 -*-
"""前两问补充分析：交会几何热图、检测点个数收敛、参数敏感性与数值收敛性。

产出四类证据，供论文"结果分析与检验"使用：
1. 定位区域直径关于（测站距离 d, 交会角 gamma）的热图；
2. 定位区域直径随检测点个数 m 的收敛曲线（均匀布局 + 随机布局对照）；
3. 问题二对最小有效接收半径、候选阈值的敏感性；
4. 问题二离散化（网格步长、误差采样）的数值收敛性。
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from .base_models import locate
from .config import ROOT_DIR
from .problem2_solution import Problem2Config, solve_problem2

SUPP_FIG_DIR = ROOT_DIR / 'figures' / 'Supplement'
SUPP_OUT_DIR = ROOT_DIR / 'output' / 'Supplement'

ANGLE_ERROR_DEG = 1.0


# ---------------------------------------------------------------- 几何核心

def symmetric_diameter(distance_m: float, gamma_deg: float,
                       error_deg: float = ANGLE_ERROR_DEG) -> float:
    """对称双站交会的定位区域直径。

    干扰源置于原点，两个检测点与干扰源距离均为 distance_m，
    从干扰源看两检测点的夹角为 gamma_deg（即交会角）。无界时返回 inf。
    """
    half = np.deg2rad(gamma_deg) / 2.0
    angles = np.array([np.pi / 2 + half, np.pi / 2 - half])
    stations = distance_m * np.column_stack((np.cos(angles), np.sin(angles)))
    observations = []
    for station in stations:
        bearing = np.rad2deg(np.arctan2(-station[1], -station[0])) % 360.0
        observations.append([station[0], station[1], bearing])
    result = locate(observations, error_deg)
    if result.status in ('empty', 'unbounded'):
        return float('inf')
    return float(result.diameter)


def ring_diameter(radius_m: float, bearings_from_source_deg,
                  error_deg: float = ANGLE_ERROR_DEG) -> float:
    """干扰源在原点、m 个检测点位于半径 radius_m 的指定方位时的定位区域直径。"""
    bearings = np.deg2rad(np.asarray(bearings_from_source_deg, dtype=float))
    stations = radius_m * np.column_stack((np.cos(bearings), np.sin(bearings)))
    observations = []
    for station in stations:
        bearing = np.rad2deg(np.arctan2(-station[1], -station[0])) % 360.0
        observations.append([station[0], station[1], bearing])
    result = locate(observations, error_deg)
    if result.status in ('empty', 'unbounded'):
        return float('inf')
    return float(result.diameter)


# ---------------------------------------------------------------- 分析一：热图

def geometry_heatmap(d_values, gamma_values):
    """计算定位直径关于（距离, 交会角）的网格。"""
    grid = np.empty((len(d_values), len(gamma_values)))
    for i, d in enumerate(d_values):
        for j, gamma in enumerate(gamma_values):
            grid[i, j] = symmetric_diameter(float(d), float(gamma))
    return grid


def best_gamma_curve(d_values, gamma_values, grid):
    """每个距离下的最优交会角。"""
    out = []
    for i, d in enumerate(d_values):
        j = int(np.nanargmin(np.where(np.isfinite(grid[i]), grid[i], np.inf)))
        out.append((float(d), float(gamma_values[j]), float(grid[i, j])))
    return out


# ---------------------------------------------------------------- 分析二：检测点个数

def station_count_curve(radius_m: float, counts, trials: int = 400, seed: int = 20260910):
    """均匀布局与随机布局下，定位直径随检测点个数的变化。"""
    rng = np.random.default_rng(seed)
    uniform, random_median = [], []
    for m in counts:
        uniform.append(ring_diameter(radius_m, np.arange(m) * 360.0 / m))
        samples = []
        for _ in range(trials):
            bearings = np.sort(rng.uniform(0, 360, m))
            value = ring_diameter(radius_m, bearings)
            if np.isfinite(value):
                samples.append(value)
        random_median.append(float(np.median(samples)) if samples else float('inf'))
    return np.array(uniform), np.array(random_median)


# ---------------------------------------------------------------- 分析三：敏感性

def sensitivity_receive_radius(radii, station1=(0.0, 0.0), bearing1=0.0):
    """最小有效接收半径对问题二结果的影响。"""
    rows = []
    for r in radii:
        config = Problem2Config(min_receive_radius_m=float(r))
        started = time.time()
        result = solve_problem2(station1, bearing1, config)
        move = float(np.linalg.norm(result.selected_point - np.asarray(station1)))
        rows.append({
            'min_receive_radius_m': float(r),
            'coverage_margin_m': float(result.coverage_margin_m),
            'reliable_point_count': int(len(result.reliable_points)),
            'good_point_count': int(len(result.good_points)),
            'min_quality_m': float(result.min_quality_m),
            'selected_station2_m': [float(v) for v in result.selected_point],
            'move_distance_m': move,
            'elapsed_s': round(time.time() - started, 2),
        })
    return rows


def sensitivity_quality_gap(gaps):
    """候选区域相对偏差系数 rho 对候选点数量的影响。"""
    rows = []
    for rho in gaps:
        config = Problem2Config(relative_quality_gap=float(rho))
        result = solve_problem2((0.0, 0.0), 0.0, config)
        rows.append({
            'rho': float(rho),
            'threshold_m': float((1.0 + rho) * result.min_quality_m),
            'good_point_count': int(len(result.good_points)),
            'selected_station2_m': [float(v) for v in result.selected_point],
        })
    return rows


def convergence_grid(settings):
    """不同离散化设置下的数值收敛性。"""
    rows = []
    for radial, angle, boundary, error_step in settings:
        errors = tuple(np.round(np.arange(-1.0, 1.0 + error_step * 0.5, error_step), 6).tolist())
        config = Problem2Config(radial_step_m=radial, angle_step_deg=angle,
                                boundary_step_m=boundary, error_samples_deg=errors)
        started = time.time()
        result = solve_problem2((0.0, 0.0), 0.0, config)
        rows.append({
            'radial_step_m': radial,
            'angle_step_deg': angle,
            'boundary_step_m': boundary,
            'error_sample_count': len(errors),
            'coverage_margin_m': float(result.coverage_margin_m),
            'possible_point_count': int(len(result.possible_points)),
            'min_quality_m': float(result.min_quality_m),
            'selected_station2_m': [float(v) for v in result.selected_point],
            'elapsed_s': round(time.time() - started, 2),
        })
    return rows


# ---------------------------------------------------------------- 绘图

def _apply_style():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'font.sans-serif': ['Microsoft YaHei', 'SimHei', 'DejaVu Sans'],
        'axes.unicode_minus': False, 'svg.fonttype': 'none',
        'axes.spines.top': False, 'axes.spines.right': False, 'font.size': 9,
    })
    return plt


def plot_heatmap(d_values, gamma_values, grid, fig_dir: Path):
    plt = _apply_style()
    finite = grid[np.isfinite(grid)]
    vmax = float(np.percentile(finite, 98)) if finite.size else 1.0
    fig, ax = plt.subplots(figsize=(7.4, 5.2), layout='constrained')
    mesh = ax.pcolormesh(gamma_values, d_values, np.clip(grid, 0, vmax),
                         cmap='viridis_r', shading='auto', vmin=0, vmax=vmax)
    best = [best_gamma_curve(d_values, gamma_values, grid)[i][1]
            for i in range(len(d_values))]
    ax.plot(best, d_values, color='#C44E52', linewidth=1.6, linestyle='--',
            label='各距离下的最优交会角')
    ax.set(xlabel=r'交会角 $\gamma$ ($^\circ$)', ylabel='检测点到干扰源距离 $d$ (m)',
           title='图A 定位区域直径随距离与交会角的变化')
    fig.colorbar(mesh, ax=ax, label='定位区域直径 (m，截断至 98% 分位)')
    ax.legend(loc='lower right', fontsize=8)
    fig_dir.mkdir(parents=True, exist_ok=True)
    for ext in ('png', 'svg'):
        fig.savefig(fig_dir / f'heatmap_geometry.{ext}', dpi=300)
    plt.close(fig)


def plot_sensitivity(rows, fig_dir: Path):
    """接收半径敏感性：定位质量与移动代价的权衡。"""
    plt = _apply_style()
    radii = [r['min_receive_radius_m'] for r in rows]
    quality = [r['min_quality_m'] for r in rows]
    move = [r['move_distance_m'] for r in rows]
    fig, ax = plt.subplots(figsize=(6.8, 4.6), layout='constrained')
    ax.plot(radii, quality, 'o-', color='#0F4D92', label='最坏定位直径 $J_{\\min}$')
    ax.set(xlabel='假设的最小有效接收半径 (m)', ylabel='$J_{\\min}$ (m)',
           title='图C 第二检测点选择的精度—代价权衡',
           xticks=radii)
    for x, y in zip(radii, quality):
        ax.annotate(f'{y:.1f}', (x, y), xytext=(0, 8), textcoords='offset points',
                    ha='center', fontsize=8, color='#0F4D92')
    ax2 = ax.twinx()
    ax2.plot(radii, move, 's--', color='#9A4D8E', label='移动到第二点的代价')
    ax2.set_ylabel('移动距离 (m)', color='#9A4D8E')
    ax2.tick_params(axis='y', colors='#9A4D8E')
    for x, y in zip(radii, move):
        ax2.annotate(f'{y:.0f}', (x, y), xytext=(0, -14), textcoords='offset points',
                     ha='center', fontsize=8, color='#9A4D8E')
    handles = ax.get_legend_handles_labels()[0] + ax2.get_legend_handles_labels()[0]
    labels = ax.get_legend_handles_labels()[1] + ax2.get_legend_handles_labels()[1]
    ax.legend(handles, labels, loc='upper center', fontsize=8)
    fig_dir.mkdir(parents=True, exist_ok=True)
    for ext in ('png', 'svg'):
        fig.savefig(fig_dir / f'sensitivity_receive_radius.{ext}', dpi=300)
    plt.close(fig)


def plot_station_curve(counts, uniform, random_median, fig_dir: Path):
    plt = _apply_style()
    fig, ax = plt.subplots(figsize=(6.8, 4.8), layout='constrained')
    ax.semilogy(counts, uniform, 'o-', color='#0F4D92', label='均匀布站')
    ax.semilogy(counts, random_median, 's--', color='#9A4D8E', label='随机布站（中位数）')
    # m=2 的均匀布站恰为两站共线、源居中的退化情形，单独标注以防误读
    if len(counts) and counts[0] == 2:
        ax.annotate('两站共线（退化）', xy=(2, uniform[0]), xytext=(2.4, uniform[0] * 0.55),
                    fontsize=8, color='#0F4D92',
                    arrowprops=dict(arrowstyle='->', color='#0F4D92', linewidth=0.8))
    ax.axhline(20.0, color='#B64342', linestyle=':', linewidth=1.2,
               label='清除半径 20 m（能否一次清除的门槛）')
    ax.set(xlabel='检测点个数 $m$', ylabel='定位区域直径 (m，对数坐标)',
           title='图B 定位区域直径随检测点个数的收敛（测站距源 800 m）',
           xticks=list(counts))
    ax.grid(alpha=0.25, linewidth=0.6, which='both')
    ax.legend(loc='upper right', fontsize=8)
    fig_dir.mkdir(parents=True, exist_ok=True)
    for ext in ('png', 'svg'):
        fig.savefig(fig_dir / f'curve_station_count.{ext}', dpi=300)
    plt.close(fig)


# ---------------------------------------------------------------- 主流程

def run(quick: bool = False):
    SUPP_FIG_DIR.mkdir(parents=True, exist_ok=True)
    SUPP_OUT_DIR.mkdir(parents=True, exist_ok=True)
    payload = {}

    d_values = np.linspace(100, 1500, 25)
    gamma_values = np.linspace(10, 170, 33)
    grid = geometry_heatmap(d_values, gamma_values)
    best = best_gamma_curve(d_values, gamma_values, grid)
    # 理论式：对称双站、交会角 90° 时 D = 2*sqrt(2)*eps*d（eps 以弧度计）
    theory = [float(2 * np.sqrt(2) * np.deg2rad(ANGLE_ERROR_DEG) * d) for d in d_values]
    payload['heatmap'] = {
        'distance_m': [float(v) for v in d_values],
        'gamma_deg': [float(v) for v in gamma_values],
        'diameter_m': [[float(v) for v in row] for row in grid],
        'best_gamma': best,
        'theory_diameter_m': theory,
        'theory_max_abs_error_m': max(abs(t - b[2]) for t, b in zip(theory, best)),
        'theory_formula': 'D = 2*sqrt(2)*eps*d  (eps = 1 deg, 对称双站, 交会角 90 deg)',
    }
    plot_heatmap(d_values, gamma_values, grid, SUPP_FIG_DIR)

    counts = [2, 3, 4, 5, 6, 8]
    uniform, random_median = station_count_curve(800.0, counts,
                                                 trials=120 if quick else 400)
    payload['station_count'] = {
        'radius_m': 800.0,
        'counts': list(counts),
        'uniform_diameter_m': [float(v) for v in uniform],
        'random_median_diameter_m': [float(v) for v in random_median],
    }
    plot_station_curve(counts, uniform, random_median, SUPP_FIG_DIR)

    payload['sensitivity_receive_radius'] = sensitivity_receive_radius(
        (1000, 1200, 1500) if not quick else (1000,))
    if len(payload['sensitivity_receive_radius']) > 1:
        plot_sensitivity(payload['sensitivity_receive_radius'], SUPP_FIG_DIR)
    payload['sensitivity_quality_gap'] = sensitivity_quality_gap((0.02, 0.05, 0.10, 0.20))
    payload['convergence'] = convergence_grid([
        (75.0, 0.5, 15.0, 1.0),
        (50.0, 0.4, 10.0, 0.5),
        (37.5, 0.3, 10.0, 0.25),
    ])

    target = SUPP_OUT_DIR / 'supplement_results.json'
    target.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
    _write_markdown(payload)
    print(target)
    return payload


def _write_markdown(payload):
    lines = ['# 前两问补充分析数值结果', '']
    heat = payload['heatmap']
    lines += ['## 1. 交会几何（图A）', '',
              '理论式：对称双站、交会角 90° 时 `D = 2√2·ε·d`（ε=1°=0.01745 rad）。', '',
              '| 距离 d (m) | 最优交会角 (°) | 实测最小直径 (m) | 理论值 (m) | 相对偏差 |',
              '| ---: | ---: | ---: | ---: | ---: |']
    step = max(1, len(heat['best_gamma']) // 7)
    for (d, gamma, dia), theo in list(zip(heat['best_gamma'],
                                          heat['theory_diameter_m']))[::step]:
        rel = abs(theo - dia) / theo * 100 if theo else 0.0
        lines.append(f'| {d:.0f} | {gamma:.1f} | {dia:.3f} | {theo:.3f} | {rel:.2f}% |')
    lines += ['', f"实测与理论式最大绝对偏差：**{heat['theory_max_abs_error_m']:.4f} m**。", '']
    lines += ['', '## 2. 检测点个数（图B，测站距源 800 m）', '',
              '| m | 均匀布站直径 (m) | 随机布站中位数 (m) |', '| ---: | ---: | ---: |']
    for m, u, r in zip(payload['station_count']['counts'],
                       payload['station_count']['uniform_diameter_m'],
                       payload['station_count']['random_median_diameter_m']):
        lines.append(f'| {m} | {u:.3f} | {r:.3f} |')
    lines += ['', '## 3. 问题二对最小有效接收半径的敏感性', '',
              '| 最小接收半径 (m) | 覆盖裕量 (m) | 可靠点数 | 候选点数 | J_min (m) | 推荐第二点 (m) | 移动距离 (m) |',
              '| ---: | ---: | ---: | ---: | ---: | --- | ---: |']
    for row in payload['sensitivity_receive_radius']:
        p = row['selected_station2_m']
        lines.append(f"| {row['min_receive_radius_m']:.0f} | {row['coverage_margin_m']:.2f} | "
                     f"{row['reliable_point_count']} | {row['good_point_count']} | "
                     f"{row['min_quality_m']:.3f} | ({p[0]:.1f}, {p[1]:.1f}) | {row['move_distance_m']:.1f} |")
    lines += ['', '## 4. 问题二对候选阈值的敏感性', '',
              '| ρ | 阈值 (m) | 候选点数 | 推荐第二点 (m) |', '| ---: | ---: | ---: | --- |']
    for row in payload['sensitivity_quality_gap']:
        p = row['selected_station2_m']
        lines.append(f"| {row['rho']:.2f} | {row['threshold_m']:.3f} | "
                     f"{row['good_point_count']} | ({p[0]:.1f}, {p[1]:.1f}) |")
    lines += ['', '## 5. 数值收敛性', '',
              '| 径向步长 (m) | 角向步长 (°) | 误差采样数 | 覆盖裕量 (m) | J_min (m) | 推荐第二点 (m) | 耗时 (s) |',
              '| ---: | ---: | ---: | ---: | ---: | --- | ---: |']
    for row in payload['convergence']:
        p = row['selected_station2_m']
        lines.append(f"| {row['radial_step_m']} | {row['angle_step_deg']} | "
                     f"{row['error_sample_count']} | {row['coverage_margin_m']:.2f} | "
                     f"{row['min_quality_m']:.3f} | ({p[0]:.1f}, {p[1]:.1f}) | {row['elapsed_s']} |")
    lines.append('')
    (SUPP_OUT_DIR / 'supplement_tables.md').write_text('\n'.join(lines), encoding='utf-8')


def replot():
    """从已保存的 JSON 结果重绘全部图形与表格，不重跑计算。"""
    data = json.loads((SUPP_OUT_DIR / 'supplement_results.json').read_text(encoding='utf-8'))
    heat = data['heatmap']
    plot_heatmap(np.array(heat['distance_m']), np.array(heat['gamma_deg']),
                 np.array(heat['diameter_m']), SUPP_FIG_DIR)
    counts = data['station_count']['counts']
    plot_station_curve(counts, np.array(data['station_count']['uniform_diameter_m']),
                       np.array(data['station_count']['random_median_diameter_m']), SUPP_FIG_DIR)
    if len(data['sensitivity_receive_radius']) > 1:
        plot_sensitivity(data['sensitivity_receive_radius'], SUPP_FIG_DIR)
    _write_markdown(data)
    print(SUPP_FIG_DIR)
    return data


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--quick', action='store_true', help='减少随机试验次数')
    parser.add_argument('--replot', action='store_true', help='从已有 JSON 重绘图，不重算')
    arguments = parser.parse_args()
    if arguments.replot:
        replot()
    else:
        run(quick=arguments.quick)
