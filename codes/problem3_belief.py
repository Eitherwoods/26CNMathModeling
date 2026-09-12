# -*- coding: utf-8 -*-
"""问题三：贝叶斯置信图层——最优搜索论的"排序与下注"层。

理论骨架
--------
Koopman (1946) / Stone (1975) 的最优搜索论：对每个频道的可能位置维护后验
概率分布（置信图），以"期望每秒清除的概率质量"给动作排序，以
"B(p,20) 内后验质量 ≥ 阈值"作为试探清除（下注）的判据。

与证书层的分工（红线约束）
--------------------------
1. 置信层**只用于排序与下注**，不产生任何严格结论。频道排除、可证清除、
   保证性追踪与结束判据仍完全由 `problem3_model` 的最坏情形掩码给出；
2. 每次读数先由证书掩码做最坏情形收缩，置信层再在其上做贝叶斯加权并归一。
   置信图的支持集恒为证书掩码的子集，因此概率为零只意味着"排序时不再押注"，
   不会反过来放大成任何"不存在/已清除"结论；
3. 探测性动作的失败代价（3 s 清除失败、5 s 无信号检测）由模拟器结算并回写
   证书掩码，置信层不承担任何保证。

似然模型（显式假设，仅服务排序与下注）
--------------------------------------
1. 示向度：报告角 = 真实方位角 + 独立误差，|误差| ≤ ANGLE_ERROR_DEG（证书
   用该最坏界）；真实演练实测残差约 0.42°。置信层用均值为 0、标准差
   `direction_std_deg` 的角度高斯核对支持集加权——它比"楔形内均匀"更接近
   实际误差分布，使沿示向线中轴的位置质量更高。
2. 接收半径：每个源的有效接收半径 a 未知但固定，先验 a ~ U[1000, 1500] m。
   "无信号"读数在距离 d 处的似然为 P(a < d)：d ≤ 1000 时为 0（与证书口径
   一致），d ∈ (1000, 1500) 时线性增长，d ≥ 1500 时为 1；"示向度"意味着
   d ≤ a，距离权重取补事件 P(a ≥ d)。近场与清除失败的排除是确定性的，
   置信层只随证书掩码清零相应圆域。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .config import (CLEAR_RADIUS_M, MAX_RECEIVE_RADIUS_M,
                     MIN_RECEIVE_RADIUS_M, NEAR_DISTANCE_M, TARGET_RADIUS_M)
from .problem3_model import Lattice

_EPS = 1e-15


def _wrap_deg(delta):
    """把角度差规范到 [-180, 180)。"""
    return (np.asarray(delta, dtype=float) + 180.0) % 360.0 - 180.0


def _uniform_disk_mass(lattice):
    """目标圆域上的均匀先验：圆域外格点质量为 0，圆域内等权。"""
    inside = np.hypot(lattice.points[:, 0], lattice.points[:, 1]) <= TARGET_RADIUS_M + 1e-9
    mass = np.where(inside, 1.0, 0.0)
    total = mass.sum()
    return mass / total if total > _EPS else mass


@dataclass
class ChannelBelief:
    """单个频道的后验置信图（细格网 + 粗格网各一份）。

    质量数组只在与证书掩码相交处非零，且恒归一（和为 1）；掩码收缩后若
    质量在数值上完全耗尽，则退回掩码上的均匀分布，保证查询仍有定义。
    """

    fine: Lattice
    coarse: Lattice
    direction_std_deg: float
    fine_mass: np.ndarray = field(init=False)
    coarse_mass: np.ndarray = field(init=False)

    def __post_init__(self):
        self.fine_mass = _uniform_disk_mass(self.fine)
        self.coarse_mass = _uniform_disk_mass(self.coarse)

    # ------------------------------------------------------------------ 更新

    def observe_direction(self, knowledge, x, y, bearing_deg):
        """示向度读数：角度高斯核 × 接收半径补事件权重，再按证书掩码归一。"""
        self.fine_mass *= self._direction_weight(self.fine, x, y, bearing_deg)
        self.coarse_mass *= self._direction_weight(self.coarse, x, y, bearing_deg)
        self._resync(knowledge)

    def observe_near(self, knowledge, x, y):
        """距离过近：证书掩码直接收缩到 B(x,5)，置信层随之重归一。"""
        self._resync(knowledge)

    def observe_no_signal(self, knowledge, x, y):
        """无信号读数：按 P(a < d) 给 (1000,1500] 环带降权（d ≤ 1000 处证书已删）。"""
        self.fine_mass *= self._no_signal_weight(self.fine, x, y)
        self.coarse_mass *= self._no_signal_weight(self.coarse, x, y)
        self._resync(knowledge)

    def observe_clear_failure(self, knowledge, x, y):
        """清除失败：B(x,20) 内确定无目标，权重置零后按证书掩码归一。"""
        for lattice, mass in ((self.fine, self.fine_mass), (self.coarse, self.coarse_mass)):
            distance = lattice.distances_to(x, y)
            mass *= distance > CLEAR_RADIUS_M
        self._resync(knowledge)

    def mark_cleared(self):
        """清除成功：该频道不再参与任何排序，质量清空。"""
        self.fine_mass = np.zeros_like(self.fine_mass)
        self.coarse_mass = np.zeros_like(self.coarse_mass)

    # ------------------------------------------------------------------ 查询

    def mass_within(self, x, y, radius_m):
        """B((x,y), radius) 内的后验质量（按格网覆盖半径保守内缩）。

        与 `ChannelKnowledge.clear_probability` 同口径：只有距 (x,y) 不超过
        radius - r_h 的格点才能保证其整个代表单元落在清除圆内。
        """
        inner = radius_m - self.fine.covering_radius_m
        if inner <= 0:
            return 0.0
        distance = self.fine.distances_to(x, y)
        return float(self.fine_mass[distance <= inner].sum())

    def mass_excluded(self, reach):
        """检测站点可排除的后验质量（reach 为细格网布尔掩码）。

        质量在证书掩码之外恒为零，因此这里无需再与掩码求交。
        """
        return float(self.fine_mass[reach].sum())

    def expected_distance_m(self, waypoint):
        """粗格网上的后验期望剩余距离（风险敏感评分的基础量）。"""
        total = float(self.coarse_mass.sum())
        if total <= _EPS:
            return 0.0
        distance = self.coarse.distances_to(float(waypoint[0]), float(waypoint[1]))
        return float(np.dot(self.coarse_mass, distance) / total)

    def quantile_distance_m(self, waypoint, quantile):
        """粗格网后验剩余距离的加权分位数。

        `quantile=1` 退化为"最坏可能距离"（与证书层 `max_distance_m` 同向），
        `quantile=0` 退化为最近可能位置；默认追击评分用中间分位数在
        "低概率长尾"与"最坏情形"之间取得风险敏感的折中。
        """
        total = float(self.coarse_mass.sum())
        if total <= _EPS:
            return 0.0
        distance = self.coarse.distances_to(float(waypoint[0]), float(waypoint[1]))
        order = np.argsort(distance)
        cumulative = np.cumsum(self.coarse_mass[order]) / total
        index = int(np.searchsorted(cumulative, min(max(float(quantile), 0.0), 1.0),
                                    side='left'))
        return float(distance[order[min(index, len(order) - 1)]])

    # ------------------------------------------------------------------ 内部

    def _direction_weight(self, lattice, x, y, bearing_deg):
        offset = lattice.points - np.array([float(x), float(y)])
        distance = np.linalg.norm(offset, axis=1)
        theta = np.degrees(np.arctan2(offset[:, 1], offset[:, 0]))
        deviation = _wrap_deg(theta - bearing_deg)
        std = max(float(self.direction_std_deg), 1e-6)
        weight = np.exp(-0.5 * (deviation / std) ** 2)
        # 示向读数要求 d ≤ a：a ~ U[1000,1500] 下距离权重为 P(a ≥ d)。
        span = MAX_RECEIVE_RADIUS_M - MIN_RECEIVE_RADIUS_M
        weight *= np.clip((MAX_RECEIVE_RADIUS_M - distance) / span, 0.0, 1.0)
        # d < 5 m 时读到的是近场而非示向度；证书掩码同样不含该圆域。
        weight = np.where(distance < NEAR_DISTANCE_M, 0.0, weight)
        return weight

    @staticmethod
    def _no_signal_weight(lattice, x, y):
        """无信号读数的似然 P(a < d)：越接近接收半径下界的位置越不可能。"""
        distance = lattice.distances_to(float(x), float(y))
        span = MAX_RECEIVE_RADIUS_M - MIN_RECEIVE_RADIUS_M
        return np.clip((distance - MIN_RECEIVE_RADIUS_M) / span, 0.0, 1.0)

    def _resync(self, knowledge):
        """按证书掩码收缩支持集并归一；质量耗尽时退回掩码上的均匀分布。"""
        self.fine_mass = self._normalize(self.fine_mass, knowledge.mask)
        self.coarse_mass = self._normalize(self.coarse_mass, knowledge.plan_mask)

    @staticmethod
    def _normalize(mass, support):
        mass = mass * support
        total = float(mass.sum())
        if total > _EPS:
            return mass / total
        # 数值耗尽的退化保护：退回证书掩码上的均匀分布（仍不越过证书支持集）。
        uniform = support.astype(float)
        count = float(uniform.sum())
        return uniform / count if count > _EPS else np.zeros_like(uniform)
