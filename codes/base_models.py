# -*- coding: utf-8 -*-
"""纯示向角域交会几何：半平面构造、区域分类、顶点和直径。

坐标单位为米，角度为从东向逆时针旋转的度数。
不将接收半径、目标圆域或近场无读数条件混入多边形定义。
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
