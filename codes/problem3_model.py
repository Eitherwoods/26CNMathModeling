# -*- coding: utf-8 -*-
"""问题三：全向干扰源可能位置集合的栅格表示与几何判据。

设计要点
--------
1. 每个频道的可能位置集合 F_c 用布尔栅格掩码表示。响应更新同时包含集合交
   （示向角域、近场圆）与集合差（无信号圆、清除失败圆），F_c 因此一般不是
   凸集，不能沿用问题一"顶点枚举 + 直径"的路线；本模块直接对掩码做外极值查询。
2. 掩码始终取真实可能集合的 r_h 外邻域（r_h = h*sqrt(2)/2 为格网覆盖半径），
   且格网点覆盖整个目标圆域。由此得到两条可用的严格结论：
       掩码为空            => 该频道在目标区域内确实不存在干扰源；
       max|q - p| + r_h <= d => 任一可能位置都落在 p 的 d 邻域内。
   所有排除类更新按相反方向保守处理（只删除确定为不可能的格点），
   保证上述结论不会因离散化而失效。
3. 细格网承担全部严格判据；粗格网只用于前瞻评分，不产生任何结论。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .base_models import distance_to_wedge, min_enclosing_circle
from .config import (ANGLE_ERROR_DEG, CLEAR_RADIUS_M, MAX_RECEIVE_RADIUS_M,
                     MIN_RECEIVE_RADIUS_M, NEAR_DISTANCE_M, TARGET_RADIUS_M)

COVERING_RADIUS_FACTOR = float(np.sqrt(2.0) / 2.0)
# 七点证书骨架的环半径：R·cos30° 使"环向中缝"的最坏漏检取最小值 R·sin30°。
# 对 R=1800 即 1558.85 m 处最坏漏检 900 m ≤ 1000 m（有效接收半径下界），
# 因此"该七点全无信号 ⇒ 频道不存在"成立。六圆覆盖半径的已知极限约
# 1798.87 m < 1800 m，故 7 是完备证书的理论最少站数（修正 oracle §11 采用）。
CERT_RING_RADIUS_M = float(900.0 * np.sqrt(3.0))


@dataclass(frozen=True)
class Lattice:
    """覆盖目标圆域的规则格网；covering_radius_m 为格网点到任意点的最大距离。"""

    points: np.ndarray
    spacing_m: float
    extent_m: float
    covering_radius_m: float

    @classmethod
    def build(cls, spacing_m, region_radius_m=TARGET_RADIUS_M):
        """构造覆盖半径 region_radius_m 圆域（含覆盖半径外扩）的格网。"""
        spacing = float(spacing_m)
        if not np.isfinite(spacing) or spacing <= 0:
            raise ValueError('格网步长必须为正的有限值。')
        covering = spacing * COVERING_RADIUS_FACTOR
        extent = float(np.ceil((float(region_radius_m) + covering) / spacing) * spacing)
        axis = np.arange(-extent, extent + spacing * 0.5, spacing)
        xx, yy = np.meshgrid(axis, axis, indexing='xy')
        points = np.column_stack((xx.ravel(), yy.ravel()))
        points = points[np.hypot(points[:, 0], points[:, 1]) <= extent + 1e-9]
        return cls(points, spacing, extent, covering)

    def __len__(self):
        return len(self.points)

    def distances_to(self, x, y):
        """返回全部格点到位移 (x, y) 的距离。"""
        return np.linalg.norm(self.points - np.array([float(x), float(y)]), axis=1)


def wedge_distance(offset, bearing_deg, half_angle_deg=ANGLE_ERROR_DEG):
    """由"格点减顶点"的偏移向量直接计算到示向角域的距离。

    与 `base_models.distance_to_wedge` 等价，但省去一次坐标相减，
    供掩码更新在十万级格点上复用同一次偏移计算。
    """
    axis = np.array([np.cos(np.deg2rad(bearing_deg)), np.sin(np.deg2rad(bearing_deg))])
    normal = np.array([-axis[1], axis[0]])
    along = offset @ axis
    across = np.abs(offset @ normal)
    inside = (along >= 0) & (across <= along * np.tan(np.deg2rad(half_angle_deg)) + 1e-12)
    edges = []
    for sign in (-1.0, 1.0):
        angle = np.deg2rad(bearing_deg + sign * half_angle_deg)
        direction = np.array([np.cos(angle), np.sin(angle)])
        projection = np.maximum(offset @ direction, 0.0)
        edges.append(np.linalg.norm(offset - projection[:, None] * direction, axis=1))
    return np.where(inside, 0.0, np.minimum(edges[0], edges[1]))


def _offset(lattice, x, y):
    """返回全部格点相对 (x, y) 的偏移向量。"""
    return lattice.points - np.array([float(x), float(y)])


def apply_direction(mask, lattice, x, y, bearing_deg):
    """示向度读数的完整更新：示向角域 ∩ (5, 1500] 距离区间，均按覆盖半径外扩。"""
    offset = _offset(lattice, x, y)
    distance = np.linalg.norm(offset, axis=1)
    keep = (wedge_distance(offset, bearing_deg) <= lattice.covering_radius_m)
    inner = NEAR_DISTANCE_M - lattice.covering_radius_m
    if inner > 0:
        keep &= distance >= inner
    keep &= distance <= MAX_RECEIVE_RADIUS_M + lattice.covering_radius_m
    return mask & keep


def apply_wedge(mask, lattice, x, y, bearing_deg, half_angle_deg=ANGLE_ERROR_DEG):
    """示向度读数：与以 (x, y) 为顶点的示向角域相交；按覆盖半径外扩。"""
    keep = distance_to_wedge(lattice.points, (x, y), bearing_deg, half_angle_deg) <= lattice.covering_radius_m
    return mask & keep


def apply_annulus(mask, lattice, x, y,
                  inner_m=NEAR_DISTANCE_M, outer_m=MAX_RECEIVE_RADIUS_M):
    """示向度读数同时给出的距离区间 (inner, outer]；两侧按覆盖半径外扩。"""
    distance = lattice.distances_to(x, y)
    keep = ((distance >= inner_m - lattice.covering_radius_m)
            & (distance <= outer_m + lattice.covering_radius_m))
    return mask & keep


def apply_near(mask, lattice, x, y, radius_m=NEAR_DISTANCE_M):
    """距离过近：直接光学清除，位置集合收缩到 B(x, 5)；按覆盖半径外扩。"""
    return mask & (lattice.distances_to(x, y) <= radius_m + lattice.covering_radius_m)


def apply_outside(mask, lattice, x, y, radius_m):
    """排除类更新：只删除可以确定为不可能的格点。

    观测只保证真实位置在半径 radius_m 之外；格点 q 若满足 |q - x| < radius - r_h，
    则任何落在 q 的 r_h 邻域内的真实位置都会违反该观测，故可安全删除。
    """
    return mask & (lattice.distances_to(x, y) >= radius_m - lattice.covering_radius_m)


def apply_distance_order(mask, lattice, signal_points, no_signal_points):
    """按未知共同接收半径施加保守的距离次序约束。

    若 S 为任一有效收信点、N 为任一无信号点，则共同未知半径 r 给出
    ``|G-S| < |G-N|``。格点代表半径为 covering_radius_m 的小单元；只有
    当 ``|q-S|-r_h >= |q-N|+r_h`` 时，q 的整个代表单元都违反该次序，
    才删除 q，从而避免严格不等式和浮点误差误删真实位置。
    """
    if not signal_points or not no_signal_points:
        return mask
    covering = float(lattice.covering_radius_m)
    keep = np.ones(len(lattice.points), dtype=bool)
    for sx, sy in signal_points:
        signal_distance = np.linalg.norm(lattice.points - np.array([sx, sy]), axis=1)
        for nx, ny in no_signal_points:
            no_signal_distance = np.linalg.norm(lattice.points - np.array([nx, ny]), axis=1)
            keep &= (signal_distance - covering < no_signal_distance + covering)
    return mask & keep


def coverage_radius_m(points, waypoints):
    """点集到最近停靠点的最大距离，用于检验扫描布站的覆盖完备性。"""
    points = np.asarray(points, dtype=float)
    waypoints = np.asarray(waypoints, dtype=float)
    distances = np.linalg.norm(points[:, None, :] - waypoints[None, :, :], axis=2)
    return float(np.max(np.min(distances, axis=1)))


def hex_waypoints(ring_radius_m, region_radius_m=TARGET_RADIUS_M):
    """中心 + 六边形的七点停靠布站（问题三扫描阶段的完备性备选方案）。"""
    angles = np.arange(6) * 60.0
    ring = np.column_stack((ring_radius_m * np.cos(np.deg2rad(angles)),
                            ring_radius_m * np.sin(np.deg2rad(angles))))
    return np.vstack((np.zeros((1, 2)), ring))


def sweep_waypoint_gain(candidates, masks, lattice, detection_radius_m):
    """候选停靠点的粗格网新增排除面积（按频道求和）。"""
    gains = np.zeros(len(candidates))
    inner = detection_radius_m - lattice.covering_radius_m
    for mask in masks:
        if not mask.any():
            continue
        points = lattice.points[mask]
        distances = np.linalg.norm(points[None, :, :] - candidates[:, None, :], axis=2)
        gains += np.sum(distances <= inner, axis=1)
    return gains * lattice.spacing_m ** 2


@dataclass
class Observation:
    """一次检测的完整记录，用于重复检测筛选与统计。"""

    x: float
    y: float
    result: str
    bearing_deg: float | None
    virtual_time_s: float


@dataclass
class ChannelKnowledge:
    """单个频道的可能位置集合、观测历史与状态。"""

    channel: int
    fine: Lattice
    coarse: Lattice
    mask: np.ndarray = field(init=False)
    plan_mask: np.ndarray = field(init=False)
    status: str = 'unknown'
    observations: list = field(default_factory=list)
    measure_positions: list = field(default_factory=list)
    clear_positions: list = field(default_factory=list)
    detected_virtual_s: float | None = None
    cleared_virtual_s: float | None = None

    def __post_init__(self):
        self.mask = np.ones(len(self.fine.points), dtype=bool)
        self.plan_mask = np.ones(len(self.coarse.points), dtype=bool)

    # ------------------------------------------------------------------ 更新

    def observe_direction(self, x, y, bearing_deg, virtual_time_s=0.0):
        """示向度读数：与示向角域及 (5, 1500] 距离区间求交。"""
        self.mask = apply_direction(self.mask, self.fine, x, y, bearing_deg)
        self.plan_mask = apply_direction(self.plan_mask, self.coarse, x, y, bearing_deg)
        self._record(x, y, 'direction', bearing_deg, virtual_time_s)
        self._apply_distance_order()

    def observe_near(self, x, y, virtual_time_s=0.0):
        """距离过近：位置集合收缩到五米圆域。"""
        self.mask = apply_near(self.mask, self.fine, x, y)
        self.plan_mask = apply_near(self.plan_mask, self.coarse, x, y)
        self._record(x, y, 'near', None, virtual_time_s)
        self._apply_distance_order()

    def observe_no_signal(self, x, y, virtual_time_s=0.0):
        """无信号：排除最小有效接收半径圆域内的位置。"""
        self.mask = apply_outside(self.mask, self.fine, x, y, MIN_RECEIVE_RADIUS_M)
        self.plan_mask = apply_outside(self.plan_mask, self.coarse, x, y, MIN_RECEIVE_RADIUS_M)
        self._record(x, y, 'no_signal', None, virtual_time_s)
        self._apply_distance_order()

    def observe_clear_failure(self, x, y, virtual_time_s=0.0):
        """清除失败：目标不在清除半径内，排除该圆域。"""
        self.clear_positions.append((float(x), float(y)))
        self.mask = apply_outside(self.mask, self.fine, x, y, CLEAR_RADIUS_M)
        self.plan_mask = apply_outside(self.plan_mask, self.coarse, x, y, CLEAR_RADIUS_M)

    def mark_cleared(self, virtual_time_s=None):
        """清除成功：位置集合清空并标记为已清除。"""
        self.status = 'cleared'
        self.cleared_virtual_s = virtual_time_s
        self.mask = np.zeros(len(self.fine.points), dtype=bool)
        self.plan_mask = np.zeros(len(self.coarse.points), dtype=bool)

    def _record(self, x, y, result, bearing_deg, virtual_time_s):
        self.observations.append(Observation(float(x), float(y), result, bearing_deg,
                                            float(virtual_time_s)))
        self.measure_positions.append((float(x), float(y)))
        # 距离过近同样是"已发现该干扰源"的证据：它给出 B(x,5) 的强约束，
        # 且按 §3.5 第 2 条应当就地清除。若只认示向度，仅在近场取到读数的频道
        # 会被清除候选完全跳过（真实缺陷，见 test_problem3 的 near 回归测试）。
        if result != 'no_signal' and self.status == 'unknown':
            self.status = 'detected'
            self.detected_virtual_s = float(virtual_time_s)

    def _apply_distance_order(self):
        """用全部历史收信/无信号点同步收紧两套可能位置掩码。"""
        signal_points = [(obs.x, obs.y) for obs in self.observations
                         if obs.result in ('direction', 'near')]
        no_signal_points = [(obs.x, obs.y) for obs in self.observations
                            if obs.result == 'no_signal']
        self.mask = apply_distance_order(self.mask, self.fine,
                                         signal_points, no_signal_points)
        self.plan_mask = apply_distance_order(self.plan_mask, self.coarse,
                                              signal_points, no_signal_points)

    # ------------------------------------------------------------------ 查询

    @property
    def is_inconsistent(self):
        """已取得读数却算出空掩码：矛盾状态。

        掩码为空本来意味着"该频道在目标区域内不存在"，但一个已经收到过读数的
        频道显然存在，因此这是约束不一致或数值退化的信号。此时不能宣布它不存在，
        也不能照常推进，必须单独标记（对应 §3.6 列出的第三类"不能判定完成"情形）。
        """
        return self.status == 'detected' and not bool(self.mask.any())

    @property
    def is_excluded(self):
        """掩码为空且无矛盾读数：可证明该频道在目标区域内没有干扰源。"""
        return self.status != 'detected' and not bool(self.mask.any())

    @property
    def is_active(self):
        """仍需处理的频道：未清除且未被排除（矛盾频道保持活跃，等待人工判读）。"""
        return self.status != 'cleared' and not self.is_excluded

    @property
    def possible_area_m2(self):
        return float(np.count_nonzero(self.mask)) * self.fine.spacing_m ** 2

    def max_distance_m(self, x, y):
        """任一可能位置到 (x, y) 的距离上界；掩码为空时返回 None。"""
        if self.is_excluded:
            return None
        distance = np.linalg.norm(self.fine.points[self.mask] - np.array([float(x), float(y)]), axis=1)
        return float(np.max(distance)) + self.fine.covering_radius_m

    def clear_probability(self, x, y):
        """均匀先验下"在此处清除成功"的概率估计（剩余区域上的显式假设）。"""
        total = int(np.count_nonzero(self.mask))
        if total == 0:
            return 0.0
        reachable = self.fine.distances_to(x, y)
        inner = CLEAR_RADIUS_M - self.fine.covering_radius_m
        return float(np.count_nonzero(self.mask & (reachable <= inner))) / total

    def certain_clear(self, x, y):
        """可证清除：全部可能位置都在清除半径内。"""
        limit = self.max_distance_m(x, y)
        return limit is not None and limit <= CLEAR_RADIUS_M

    def near_certain_clear(self, x, y):
        """可证清除：最近一次读数已确认目标在五米内。"""
        if not self.observations:
            return False
        latest = self.observations[-1]
        return (latest.result == 'near'
                and abs(latest.x - x) <= 1e-9 and abs(latest.y - y) <= 1e-9)

    def reachable_within(self, x, y, radius_m):
        """该频道在 (x, y) 处检测时仍可能给出信号的格点面积（粗格网）。"""
        if not self.plan_mask.any():
            return 0.0
        inner = radius_m - self.coarse.covering_radius_m
        return float(np.count_nonzero(self.plan_mask & (self.coarse.distances_to(x, y) <= inner))) \
            * self.coarse.spacing_m ** 2

    def region_estimate(self, max_points=4000, use_fine=False):
        """可能位置集合的最小覆盖圆 (center, radius)，用于选择接近方向。

        返回的是点集的几何外接半径，不含格网覆盖半径（覆盖半径在
        `max_distance_m` 里以保守上界的形式出现）。
        """
        lattice, mask = (self.fine, self.mask) if use_fine else (self.coarse, self.plan_mask)
        points = lattice.points[mask]
        if len(points) == 0:
            return None, None
        if len(points) > max_points:
            points = points[::int(np.ceil(len(points) / max_points))]
        center, _ = min_enclosing_circle(points)
        radius = float(np.max(np.linalg.norm(points - center, axis=1)))
        return center, radius

    def measured_at(self, x, y, tolerance_m=1e-6):
        """返回在同一停靠点已取得的检测记录（同点重复检测不增加信息）。"""
        for observation in self.observations:
            if np.hypot(observation.x - x, observation.y - y) <= tolerance_m:
                return observation
        return None

    def cleared_here(self, x, y, tolerance_m=1e-6):
        """返回在同一停靠点已尝试过的清除记录。"""
        return any(np.hypot(px - x, py - y) <= tolerance_m for px, py in self.clear_positions)
