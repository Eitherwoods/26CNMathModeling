# -*- coding: utf-8 -*-
"""问题二：基于保守离散化的第二检测点选择与候选区域计算。

模块将连续可能区域的边界离散化，并以覆盖半径收紧最小接收半径，
使被保留的网格检测点在给定数值分辨率下仍满足可靠接收约束。
"""
from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass

import numpy as np

from .base_models import angular_distance_deg, bearing_deg, sampled_diameter
from .config import (ANGLE_ERROR_DEG, MAX_RECEIVE_RADIUS_M, MIN_RECEIVE_RADIUS_M,
                     NEAR_DISTANCE_M, PROBLEM2_FIG_DIR, PROBLEM2_OUTPUT_DIR,
                     TARGET_RADIUS_M)
from .plotting import plot_problem2_regions


@dataclass(frozen=True)
class Problem2Config:
    """问题二数值求解参数；步长越小，保守覆盖裕量越接近连续模型。"""

    angle_step_deg: float = 0.5
    radial_step_m: float = 75.0
    candidate_step_m: float = 50.0
    boundary_step_m: float = 15.0
    error_samples_deg: tuple[float, ...] = (-1.0, 0.0, 1.0)
    relative_quality_gap: float = 0.05
    target_radius_m: float = TARGET_RADIUS_M
    min_receive_radius_m: float = MIN_RECEIVE_RADIUS_M
    max_receive_radius_m: float = MAX_RECEIVE_RADIUS_M
    near_distance_m: float = NEAR_DISTANCE_M
    angle_error_deg: float = ANGLE_ERROR_DEG


@dataclass
class Problem2Result:
    """第二检测点选择的中间集合、质量指标和最终推荐结果。"""

    station1: np.ndarray
    bearing1_deg: float
    possible_points: np.ndarray
    boundary_points: np.ndarray
    reliable_points: np.ndarray
    good_points: np.ndarray
    quality_values_m: np.ndarray
    selected_point: np.ndarray
    coverage_margin_m: float
    min_quality_m: float
    config: Problem2Config


def _unique_points(points: np.ndarray) -> np.ndarray:
    """以微米级舍入去除边界采样拼接产生的重复点。"""
    if len(points) == 0:
        return np.empty((0, 2), dtype=float)
    rounded = np.round(np.asarray(points, dtype=float), decimals=8)
    _, indices = np.unique(rounded, axis=0, return_index=True)
    return np.asarray(points, dtype=float)[np.sort(indices)]


def _in_possible_region(points: np.ndarray, station1: np.ndarray, bearing1_deg: float,
                        config: Problem2Config, *, include_inner_boundary: bool = True) -> np.ndarray:
    """判断点是否落在第一次观测后的闭包可能区域中。"""
    relative = np.asarray(points, dtype=float) - station1
    distances = np.linalg.norm(relative, axis=1)
    inner_ok = distances >= config.near_distance_m if include_inner_boundary else distances > config.near_distance_m
    return (inner_ok & (distances <= config.max_receive_radius_m + 1e-9)
            & (np.linalg.norm(points, axis=1) <= config.target_radius_m + 1e-9)
            & (angular_distance_deg(bearing_deg(relative), bearing1_deg) <= config.angle_error_deg + 1e-9))


def possible_region_samples(station1: np.ndarray, bearing1_deg: float,
                            config: Problem2Config) -> tuple[np.ndarray, np.ndarray]:
    """生成可能区域内部点及其边界点，供定位质量和可靠性判定使用。"""
    station1 = np.asarray(station1, dtype=float)
    angle_values = np.arange(bearing1_deg - config.angle_error_deg,
                             bearing1_deg + config.angle_error_deg + config.angle_step_deg * 0.5,
                             config.angle_step_deg)
    angle_values[-1] = bearing1_deg + config.angle_error_deg
    radii = np.arange(config.near_distance_m, config.max_receive_radius_m + config.radial_step_m * 0.5,
                      config.radial_step_m)
    radii[-1] = config.max_receive_radius_m
    theta, radius = np.meshgrid(np.deg2rad(angle_values), radii, indexing='ij')
    interior = station1 + np.column_stack((radius.ravel() * np.cos(theta.ravel()),
                                            radius.ravel() * np.sin(theta.ravel())))
    interior = interior[_in_possible_region(interior, station1, bearing1_deg, config)]

    # 极坐标四条边保证扇形边界被采样；目标圆边界单独采样，避免裁剪边漏检。
    boundary = []
    radial_boundary = np.arange(config.near_distance_m, config.max_receive_radius_m + config.boundary_step_m * 0.5,
                                config.boundary_step_m)
    radial_boundary[-1] = config.max_receive_radius_m
    for angle in (bearing1_deg - config.angle_error_deg, bearing1_deg + config.angle_error_deg):
        direction = np.array([np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))])
        boundary.append(station1 + radial_boundary[:, None] * direction)
    arc_angles = np.arange(bearing1_deg - config.angle_error_deg,
                           bearing1_deg + config.angle_error_deg + config.boundary_step_m / config.max_receive_radius_m * 180 / np.pi,
                           config.boundary_step_m / config.max_receive_radius_m * 180 / np.pi)
    arc_angles[-1] = bearing1_deg + config.angle_error_deg
    for radius_value in (config.near_distance_m, config.max_receive_radius_m):
        directions = np.column_stack((np.cos(np.deg2rad(arc_angles)), np.sin(np.deg2rad(arc_angles))))
        boundary.append(station1 + radius_value * directions)
    target_angles = np.arange(0.0, 360.0, config.boundary_step_m / config.target_radius_m * 180 / np.pi)
    target_boundary = config.target_radius_m * np.column_stack((np.cos(np.deg2rad(target_angles)),
                                                                  np.sin(np.deg2rad(target_angles))))
    boundary.append(target_boundary[_in_possible_region(target_boundary, station1, bearing1_deg, config)])
    boundary_points = _unique_points(np.vstack(boundary))
    boundary_points = boundary_points[_in_possible_region(boundary_points, station1, bearing1_deg, config)]
    return _unique_points(np.vstack((interior, boundary_points))), boundary_points


def coverage_margin(config: Problem2Config) -> float:
    """返回边界离散化的保守覆盖裕量，供连续可靠性约束收紧半径。"""
    polar_half_diagonal = np.hypot(config.radial_step_m / 2.0,
                                   config.max_receive_radius_m * np.deg2rad(config.angle_step_deg) / 2.0)
    return float(polar_half_diagonal + config.boundary_step_m)


def reliable_candidate_grid(boundary_points: np.ndarray, config: Problem2Config) -> np.ndarray:
    """在一个参考可能位置的接收圆内生成网格并保留可靠检测点。"""
    if len(boundary_points) == 0:
        return np.empty((0, 2), dtype=float)
    reference = boundary_points[0]
    axis = np.arange(-config.min_receive_radius_m, config.min_receive_radius_m + config.candidate_step_m * 0.5,
                     config.candidate_step_m)
    xx, yy = np.meshgrid(axis, axis, indexing='xy')
    candidates = reference + np.column_stack((xx.ravel(), yy.ravel()))
    candidates = candidates[np.linalg.norm(candidates - reference, axis=1) <= config.min_receive_radius_m + 1e-9]
    margin = coverage_margin(config)
    distances = np.linalg.norm(candidates[:, None, :] - boundary_points[None, :, :], axis=2)
    return candidates[np.max(distances, axis=1) <= config.min_receive_radius_m - margin + 1e-9]


def direction_result_points(possible_points: np.ndarray, station2: np.ndarray,
                            observed_bearing_deg: float, config: Problem2Config) -> np.ndarray:
    """按第二次示向度筛选定位点，并排除应返回“距离过近”的位置。"""
    vectors = possible_points - station2
    distances = np.linalg.norm(vectors, axis=1)
    mask = ((distances > config.near_distance_m + 1e-9)
            & (angular_distance_deg(bearing_deg(vectors), observed_bearing_deg) <= config.angle_error_deg + 1e-9))
    return possible_points[mask]


def robust_quality(station2: np.ndarray, possible_points: np.ndarray,
                   config: Problem2Config) -> float:
    """枚举可能真实位置与测向误差，计算第二检测点的最坏采样定位直径。"""
    maximum_diameter = 0.0
    for source in possible_points:
        distance = float(np.linalg.norm(source - station2))
        if distance <= config.near_distance_m:
            localization = possible_points[np.linalg.norm(possible_points - station2, axis=1)
                                           <= config.near_distance_m + 1e-9]
            maximum_diameter = max(maximum_diameter, sampled_diameter(localization))
            continue
        true_bearing = float(bearing_deg((source - station2)[None, :])[0])
        for error in config.error_samples_deg:
            localization = direction_result_points(possible_points, station2, true_bearing + error, config)
            maximum_diameter = max(maximum_diameter, sampled_diameter(localization))
    return maximum_diameter


def solve_problem2(station1: tuple[float, float] | np.ndarray, bearing1_deg: float,
                   config: Problem2Config | None = None) -> Problem2Result:
    """求取可靠候选区域与最短移动距离的第二检测点；无可行点时显式报错。"""
    config = config or Problem2Config()
    station1 = np.asarray(station1, dtype=float)
    if station1.shape != (2,) or not np.all(np.isfinite(station1)) or not np.isfinite(bearing1_deg):
        raise ValueError('第一次检测点必须是有限二维坐标，示向度必须为有限数。')
    if config.min_receive_radius_m <= coverage_margin(config):
        raise ValueError('离散步长过粗，覆盖裕量已耗尽最小接收半径。')
    possible_points, boundary_points = possible_region_samples(station1, bearing1_deg % 360.0, config)
    reliable_points = reliable_candidate_grid(boundary_points, config)
    if len(reliable_points) == 0:
        raise RuntimeError('当前网格未找到可靠检测点；请缩小候选网格步长或离散步长。')
    quality_values = np.array([robust_quality(point, possible_points, config) for point in reliable_points])
    min_quality = float(np.min(quality_values))
    good_points = reliable_points[quality_values <= (1.0 + config.relative_quality_gap) * min_quality + 1e-9]
    move_distances = np.linalg.norm(good_points - station1, axis=1)
    selected_point = good_points[int(np.argmin(move_distances))]
    return Problem2Result(station1, bearing1_deg % 360.0, possible_points, boundary_points,
                          reliable_points, good_points, quality_values, selected_point,
                          coverage_margin(config), min_quality, config)


def serialize_result(result: Problem2Result) -> dict:
    """将求解结果转换为 UTF-8 JSON 可写入的结构。"""
    return {
        'model': '问题二保守离散化最坏定位直径模型',
        'station1_m': result.station1.tolist(),
        'bearing1_deg': result.bearing1_deg,
        'coverage_margin_m': result.coverage_margin_m,
        'possible_point_count': len(result.possible_points),
        'reliable_points_m': result.reliable_points.tolist(),
        'quality_values_m': result.quality_values_m.tolist(),
        'good_points_m': result.good_points.tolist(),
        'min_quality_m': result.min_quality_m,
        'selected_station2_m': result.selected_point.tolist(),
        'config': asdict(result.config),
    }


def main() -> None:
    """解析命令行人工观测，写入问题二 JSON 结果并绘制候选区域图。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--station', nargs=2, type=float, metavar=('X', 'Y'), default=(0.0, 0.0),
                        help='第一次检测点坐标，默认 (0, 0)')
    parser.add_argument('--bearing', type=float, default=0.0, help='第一次测得示向度（度），默认 0')
    parser.add_argument('--candidate-step', type=float, default=Problem2Config.candidate_step_m,
                        help='第二检测点网格步长（米）')
    args = parser.parse_args()
    config = Problem2Config(candidate_step_m=args.candidate_step)
    result = solve_problem2(args.station, args.bearing, config)
    PROBLEM2_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    target = PROBLEM2_OUTPUT_DIR / 'problem2_results.json'
    target.write_text(json.dumps(serialize_result(result), ensure_ascii=False, indent=2), encoding='utf-8')
    plot_problem2_regions(result, PROBLEM2_FIG_DIR)
    print(f'可靠点数：{len(result.reliable_points)}，较优候选点数：{len(result.good_points)}')
    print(f'推荐第二检测点：({result.selected_point[0]:.6f}, {result.selected_point[1]:.6f}) m')
    print(target)


if __name__ == '__main__':
    main()
