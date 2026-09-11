# -*- coding: utf-8 -*-
"""公共几何：方位角、凸包、点集最小覆盖圆、示向角域距离，以及问题一的角域交会。

坐标单位为米，角度为从东向逆时针旋转的度数。
问题一的定位区域只由示向角域确定，不将接收半径、目标圆域或近场无读数条件混入多边形定义；
问题三另用"示向角域距离"与栅格掩码处理带圆域约束的可能位置集合。
"""
from __future__ import annotations
from dataclasses import dataclass
from itertools import combinations

import numpy as np
from scipy.optimize import linprog

from .config import ANGLE_ERROR_DEG, DISTANCE_TOL, PARALLEL_TOL


@dataclass
class RegionResult:
    """区域分类结果；空集直径为 None，无界区域直径为正无穷。"""
    status: str
    vertices: np.ndarray
    diameter: float | None
    endpoints: np.ndarray


def bearing_halfplanes(observations, error_deg=ANGLE_ERROR_DEG):
    """将每行 (x, y, 示向度) 转为单位法向量约束 A @ p <= b。"""
    data = np.asarray(observations, dtype=float)
    if data.ndim != 2 or data.shape[1] != 3 or len(data) == 0:
        raise ValueError('观测必须是非空的 m×3 数组。')
    if not np.all(np.isfinite(data)) or not np.isfinite(error_deg):
        raise ValueError('输入必须为有限数值。')
    if not 0 < error_deg < 90:
        raise ValueError('角度半宽必须严格介于 0 和 90 度之间。')
    lower = np.deg2rad((data[:, 2] - error_deg) % 360)
    upper = np.deg2rad((data[:, 2] + error_deg) % 360)
    # 下边界左侧与上边界右侧相交，保留射线朝向，避免反向锥域。
    normals = np.stack((np.column_stack((np.sin(lower), -np.cos(lower))),
                        np.column_stack((-np.sin(upper), np.cos(upper)))), axis=1)
    offsets = np.einsum('mki,mi->mk', normals, data[:, :2])
    return normals.reshape(-1, 2), offsets.reshape(-1)


def solve_halfplanes(normals, offsets, tol=DISTANCE_TOL):
    """分类二维闭半平面交集，枚举有限顶点并计算最大顶点距离。

    用四个坐标方向的线性规划检测有界性，变量允许取负值。
    总体枚举复杂度 O(m^3)，直径阶段 O(n^2)，适用于少量观测。
    """
    a = np.asarray(normals, dtype=float)
    b = np.asarray(offsets, dtype=float)
    if a.ndim != 2 or a.shape[1] != 2 or len(a) == 0 or b.shape != (len(a),):
        raise ValueError('半平面数组形状不正确。')
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)) or not np.isfinite(tol) or tol <= 0:
        raise ValueError('半平面和容差必须是有效有限值。')
    norms = np.linalg.norm(a, axis=1)
    if np.any(norms == 0):
        raise ValueError('半平面法向量不能为零。')
    a, b = a / norms[:, None], b / norms
    empty = np.empty((0, 2))

    def optimize(cost):
        """在原始约束上调用 HiGHS，异常状态不冒充几何结论。"""
        result = linprog(cost, A_ub=a, b_ub=b, bounds=[(None, None)] * 2,
                         method='highs', options={'primal_feasibility_tolerance': tol})
        if result.status not in (0, 2, 3):
            raise RuntimeError(f'线性规划数值失败：{result.message}')
        return result

    if optimize([0, 0]).status == 2:
        return RegionResult('empty', empty, None, empty)
    for cost in ([1, 0], [-1, 0], [0, 1], [0, -1]):
        result = optimize(cost)
        if result.status == 3:
            return RegionResult('unbounded', empty, float('inf'), empty)
        if result.status != 0:
            raise RuntimeError('可行性与有界性检查不一致。')
    vertices = []
    for i, j in combinations(range(len(a)), 2):
        matrix = a[[i, j]]
        if abs(np.linalg.det(matrix)) <= PARALLEL_TOL:
            continue
        point = np.linalg.solve(matrix, b[[i, j]])
        if np.all(a @ point <= b + tol) and not any(
                np.linalg.norm(point - old) <= tol for old in vertices):
            vertices.append(point)
    if not vertices:
        raise RuntimeError('区域有界但未提取到顶点，请检查尺度和容差。')
    vertices = np.asarray(vertices)
    center = vertices.mean(axis=0)
    order = np.argsort(np.arctan2(vertices[:, 1] - center[1], vertices[:, 0] - center[0]))
    vertices = vertices[order]
    distances = np.linalg.norm(vertices[:, None] - vertices[None, :], axis=2)
    i, j = np.unravel_index(np.argmax(distances), distances.shape)
    rank = np.linalg.matrix_rank(vertices - vertices[0], tol=tol)
    status = ('point', 'segment', 'polygon')[rank]
    return RegionResult(status, vertices, float(distances[i, j]), vertices[[i, j]])


def locate(observations, error_deg=ANGLE_ERROR_DEG):
    """从示向观测计算定位区域；不会自动添加人工包围框。"""
    return solve_halfplanes(*bearing_halfplanes(observations, error_deg))


def bearing_deg(vectors):
    """把二维向量（或向量数组）转换为 [0, 360) 度方位角。"""
    vectors = np.asarray(vectors, dtype=float)
    return np.rad2deg(np.arctan2(vectors[..., 1], vectors[..., 0])) % 360.0


def angular_distance_deg(first, second):
    """返回两个方位角之间的环形最小差值，正确处理 0 度跨越。"""
    return np.abs((np.asarray(first) - np.asarray(second) + 180.0) % 360.0 - 180.0)


def distance_to_ray(points, apex, direction):
    """点到射线的欧氏距离（射线起点计入，投影为负时取起点）。"""
    points = np.asarray(points, dtype=float)
    direction = np.asarray(direction, dtype=float)
    direction = direction / np.linalg.norm(direction)
    offset = points - np.asarray(apex, dtype=float)
    projection = np.maximum(offset @ direction, 0.0)
    return np.linalg.norm(offset - projection[:, None] * direction, axis=1)


def distance_to_wedge(points, apex, bearing, half_angle_deg):
    """点到以 apex 为顶点、张角 2*half_angle_deg 的示向角域的欧氏距离。

    角域是两条边界射线生成的凸锥，其边界由两条射线组成，
    因此锥内距离为 0，锥外距离等于到两条边界射线距离的较小者。
    """
    points = np.asarray(points, dtype=float)
    apex = np.asarray(apex, dtype=float)
    if not np.isfinite(half_angle_deg) or not 0 < half_angle_deg < 90:
        raise ValueError('角度半宽必须严格介于 0 和 90 度之间。')
    offset = points - apex
    axis = np.array([np.cos(np.deg2rad(bearing)), np.sin(np.deg2rad(bearing))])
    normal = np.array([-axis[1], axis[0]])
    along = offset @ axis
    across = np.abs(offset @ normal)
    inside = (along >= 0) & (across <= along * np.tan(np.deg2rad(half_angle_deg)) + 1e-12)
    edges = [distance_to_ray(points, apex, [np.cos(np.deg2rad(bearing + sign * half_angle_deg)),
                                            np.sin(np.deg2rad(bearing + sign * half_angle_deg))])
             for sign in (-1.0, 1.0)]
    return np.where(inside, 0.0, np.minimum(edges[0], edges[1]))


def convex_hull(points):
    """使用单调链算法返回二维点集的凸包顶点（逆时针，不含共线内点）。"""
    points = np.unique(np.asarray(points, dtype=float), axis=0)
    if len(points) <= 2:
        return points
    ordered = points[np.lexsort((points[:, 1], points[:, 0]))]

    def cross(origin, first, second):
        """计算有向面积，用于剔除凸包内侧点。"""
        first_vector = first - origin
        second_vector = second - origin
        return float(first_vector[0] * second_vector[1] - first_vector[1] * second_vector[0])

    lower: list[np.ndarray] = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 1e-10:
            lower.pop()
        lower.append(point)
    upper: list[np.ndarray] = []
    for point in ordered[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 1e-10:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1])


def sampled_diameter(points):
    """计算有限点集的直径；只枚举凸包点对以减少重复计算。"""
    hull = convex_hull(points)
    if len(hull) <= 1:
        return 0.0
    distances = np.linalg.norm(hull[:, None, :] - hull[None, :, :], axis=2)
    return float(np.max(distances))


def min_enclosing_circle(points, *, tolerance=1e-9, seed=0):
    """返回二维点集的最小覆盖圆 (center, radius)。

    最小覆盖圆只由凸包顶点决定，故先求凸包，再在凸包顶点上运行随机增量
    （Welzl）算法。这样得到的是精确解：此前"迭代逼近 + 支撑点组合修正"的版本
    在处理楔形等狭长点集时，会因支撑点筛选与实际支点不一致而给出偏大的半径
    （实测楔形掩码上偏大 18%、圆心偏差 137 m），足以影响问题三的接近方向选择。

    与问题一的直径圆的关系（Jung 定理）：
        diam/2 <= r <= diam/sqrt(3)
    即覆盖圆半径恒不小于直径的一半、不大于直径的 0.578 倍。
    因此 `K_c = {X : R_c(X) <= 20}` 既不能用半径等于直径的"直径圆"来判定
    （会过度保守），也不能用半直径圆来判定（对锐角分布会乐观失真）。
    严格的覆盖结论一律由 `ChannelKnowledge.max_distance_m` 一类的显式上界给出。
    """
    unique = np.unique(np.asarray(points, dtype=float), axis=0)
    if len(unique) == 0:
        raise ValueError('最小覆盖圆至少需要一个点。')
    if len(unique) == 1:
        return unique[0].copy(), 0.0
    hull = convex_hull(unique)
    if len(hull) <= 2:
        center = hull.mean(axis=0)
        return center, float(np.max(np.linalg.norm(unique - center, axis=1)))
    order = np.random.default_rng(seed).permutation(len(hull))
    shuffled = hull[order]
    circle = _two_point_circle(shuffled[0], shuffled[1])
    for index in range(2, len(shuffled)):
        if _inside_circle(shuffled[index], circle, tolerance):
            continue
        circle = _circle_through_boundary(shuffled[:index + 1], shuffled[index], tolerance)
    return circle[0], float(np.max(np.linalg.norm(unique - circle[0], axis=1)))


def _two_point_circle(first, second):
    """以两点为直径的圆。"""
    center = (first + second) / 2.0
    return center, float(np.linalg.norm(first - center))


def _inside_circle(point, circle, tolerance):
    """判断点是否落在圆内（含容差）。"""
    center, radius = circle
    return float(np.linalg.norm(point - center)) <= radius * (1.0 + tolerance) + tolerance


def _circle_through_boundary(points, boundary, tolerance):
    """求覆盖 points 且经过 boundary 的小圆（Welzl 第二层）。"""
    circle = _two_point_circle(points[0], boundary)
    for index in range(1, len(points)):
        if _inside_circle(points[index], circle, tolerance):
            continue
        circle = _circle_through_two_boundary(points[:index + 1], points[index], boundary, tolerance)
    return circle


def _circle_through_two_boundary(points, first, second, tolerance):
    """求覆盖 points 且经过 first、second 的小圆（Welzl 第三层）。"""
    circle = _two_point_circle(first, second)
    for index in range(len(points)):
        if _inside_circle(points[index], circle, tolerance):
            continue
        center = _circle_through(np.array([first, second, points[index]]))
        if center is None:
            # 三点接近共线时退化为最远两点确定的圆，保持覆盖性。
            far = max((first, second, points[index]),
                      key=lambda pair: float(np.linalg.norm(np.asarray(first) - np.asarray(pair))))
            circle = _two_point_circle(np.asarray(first), np.asarray(far))
            continue
        circle = (center, float(np.linalg.norm(np.asarray(first) - center)))
    return circle


def _circle_through(points):
    """返回经过 2 或 3 个点的圆圆心；退化时返回 None。"""
    if len(points) == 2:
        center = points.mean(axis=0)
        if np.linalg.norm(points[0] - points[1]) == 0:
            return None
        return center
    matrix = 2.0 * (points[1:] - points[0])
    values = (points[1:] ** 2).sum(axis=1) - (points[0] ** 2).sum()
    try:
        return np.linalg.solve(matrix, values)
    except np.linalg.LinAlgError:
        return None
