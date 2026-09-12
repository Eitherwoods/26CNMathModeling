# -*- coding: utf-8 -*-
"""问题四未知频道搜索覆盖的保守方向证据。"""
from __future__ import annotations

from typing import Hashable

import numpy as np

from .config import MIN_RECEIVE_RADIUS_M


class DirectionalCoverage:
    """以方向位图记录每个位置单元中仍可能存在的发射方向。"""

    def __init__(self, lattice, orientation_bins: int = 24):
        """校验格网和方向分箱，并初始化按站点坐标缓存。"""
        if (isinstance(orientation_bins, bool) or
                not isinstance(orientation_bins, (int, np.integer)) or
                not 1 <= int(orientation_bins) <= 32):
            raise ValueError('orientation_bins 必须是 1..32 的整数。')
        points = np.asarray(lattice.points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            raise ValueError('lattice.points 必须是有限的二维坐标。')
        rh = float(lattice.covering_radius_m)
        if not np.isfinite(rh) or rh < 0:
            raise ValueError('covering_radius_m 必须是非负有限值。')
        self.lattice = lattice
        self.orientation_bins = int(orientation_bins)
        self.rh = rh
        self._cache: dict[Hashable, np.ndarray] = {}
        angles = (np.arange(self.orientation_bins) + 0.5) * (2.0 * np.pi / self.orientation_bins)
        self._normals = np.column_stack((np.cos(angles), np.sin(angles)))
        self._full = np.uint32((1 << self.orientation_bins) - 1)

    def _point(self, point):
        """将站点坐标规范化为有限二维浮点数组。"""
        value = np.asarray(point, dtype=float)
        if value.shape != (2,) or not np.isfinite(value).all():
            raise ValueError('point 必须是有限二维坐标。')
        return value

    def new_mask(self):
        """返回所有位置单元均保留全部方向的独立掩码。"""
        return np.full(len(self.lattice.points), self._full, dtype=np.uint32)

    def station_mask(self, point):
        """计算站点对每个位置单元的可证明清除方向位图，并缓存结果。"""
        station = self._point(point)
        key = (float(station[0]), float(station[1]))
        cached = self._cache.get(key)
        if cached is not None:
            return cached.copy()
        offset = station[None, :] - np.asarray(self.lattice.points, dtype=float)
        distance = np.linalg.norm(offset, axis=1)
        # 严格 margin 确保单元边界和接收半径边界不被误排除。
        near = distance + self.rh < MIN_RECEIVE_RADIUS_M - 1e-8
        margin = self.rh + 2.0 * distance * np.sin(np.pi / (2.0 * self.orientation_bins))
        dot = offset @ self._normals.T
        guaranteed = near[:, None] & (dot > margin[:, None] + 1e-8)
        result = np.full(len(offset), self._full, dtype=np.uint32)
        for bit in range(self.orientation_bins):
            result[guaranteed[:, bit]] &= np.uint32(int(self._full) ^ (1 << bit))
        self._cache[key] = result
        return result.copy()

    def update(self, mask, point):
        """纯函数式地应用站点证据，返回新的方向掩码。"""
        value = np.asarray(mask, dtype=np.uint32)
        if value.shape != (len(self.lattice.points),):
            raise ValueError('mask 长度必须等于 lattice.points 数量。')
        return value & self.station_mask(point)

    def gain(self, mask, point):
        """返回该站点本次删除的方向位总数（支持空掩码）。"""
        value = np.asarray(mask, dtype=np.uint32)
        if value.shape != (len(self.lattice.points),):
            raise ValueError('mask 长度必须等于 lattice.points 数量。')
        after = self.update(value, point)
        before_count = int(np.unpackbits(value.view(np.uint8)).sum())
        after_count = int(np.unpackbits(after.view(np.uint8)).sum())
        return before_count - after_count
