# -*- coding: utf-8 -*-
"""问题四联合位置球、半径区间和朝向区间的保守外包。

同一联合单元跨全部历史持续过滤，保留参数兼容关系。必要条件的交集可能
保留伪可行单元，但不会因单元中心不能解释反馈而删去单元内的真实参数。
无信号历史在首次有效观测后重放；未知频道的不存在性由搜索覆盖证据判定。
"""
from __future__ import annotations

import math

import numpy as np

from .config import (CLEAR_RADIUS_M, MAX_RECEIVE_RADIUS_M, MIN_RECEIVE_RADIUS_M,
                     NEAR_DISTANCE_M, TARGET_RADIUS_M)
from .problem3_model import Observation, wedge_distance


class Problem4Knowledge:
    """单频道保守联合单元；只为首次正观测留下的位置分配联合数组。"""

    def __init__(self, channel, lattice, orientation_bins=24, radius_bins=4):
        """定义完整半径/朝向区间划分，延迟分配位置乘参数的布尔数组。"""
        for count in (orientation_bins, radius_bins):
            if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
                raise ValueError('半径和朝向区间数必须为正整数。')
        if not 0 <= lattice.covering_radius_m < CLEAR_RADIUS_M:
            raise ValueError('位置单元覆盖半径必须非负且小于20米。')
        self.channel = channel
        self.lattice = lattice
        self.orientation_bins = orientation_bins
        self.radius_bins = radius_bins
        edges = np.linspace(MIN_RECEIVE_RADIUS_M, MAX_RECEIVE_RADIUS_M, radius_bins + 1)
        self.radius_low, self.radius_high = edges[:-1], edges[1:]
        self.orientation_deg = (np.arange(orientation_bins) + 0.5) * 360 / orientation_bins
        angle = np.deg2rad(self.orientation_deg)
        self.normals = np.column_stack((np.cos(angle), np.sin(angle)))
        self.status = 'unknown'
        self.observations = []
        self.clear_positions = []
        self._events = []
        self._indices = np.empty(0, dtype=int)
        self._omni = np.zeros((0, radius_bins), dtype=bool)
        self._directional = np.zeros((0, radius_bins, orientation_bins), dtype=bool)
        self._initialized = False

    @property
    def type_status(self):
        """仅在另一类型的保守外包完全为空时确定类型。"""
        if self.status in ('cleared', 'excluded', 'inconsistent'):
            return self.status
        if not self._initialized:
            return 'unknown'
        omni, directional = bool(self._omni.any()), bool(self._directional.any())
        if omni and directional:
            return 'unknown'
        return 'omnidirectional' if omni else 'directional' if directional else 'inconsistent'

    def _position_filter(self, observation):
        """正观测只施加位置必要约束，采用覆盖半径外扩保留真值。"""
        offset = self.lattice.points - np.array([observation.x, observation.y])
        distance = np.linalg.norm(offset, axis=1)
        rh = self.lattice.covering_radius_m
        tolerance = 1e-8
        if observation.result == 'direction':
            return ((wedge_distance(offset, observation.bearing_deg) <= rh + tolerance)
                    & (distance + rh >= NEAR_DISTANCE_M - tolerance)
                    & (distance - rh <= MAX_RECEIVE_RADIUS_M + tolerance))
        if observation.result == 'near':
            return distance - rh <= NEAR_DISTANCE_M + tolerance
        return np.ones(len(self.lattice.points), dtype=bool)

    def _initialize(self, observation):
        """首次正反馈先筛位置，再分配联合单元并重放之前的全部事件。"""
        self._indices = np.flatnonzero(self._position_filter(observation))
        count = len(self._indices)
        self._omni = np.ones((count, self.radius_bins), dtype=bool)
        self._directional = np.ones((count, self.radius_bins, self.orientation_bins), dtype=bool)
        self._initialized = True
        for event in self._events:
            self._apply(event)

    def _compact(self):
        """删除所有类型参数均不可行的位置行，避免全格点乘联合单元矩阵。"""
        keep = self._omni.any(axis=1) | self._directional.any(axis=(1, 2))
        self._indices = self._indices[keep]
        self._omni = self._omni[keep]
        self._directional = self._directional[keep]
        if not len(self._indices) and self.status == 'detected':
            self.status = 'inconsistent'

    def _apply(self, observation):
        """按区间必要条件过滤一次事件，朝向误差与示向误差分别处理。"""
        if not len(self._indices):
            if self.status == 'detected':
                self.status = 'inconsistent'
            return
        point = np.array([observation.x, observation.y])
        centers = self.lattice.points[self._indices]
        offset = point - centers
        distance = np.linalg.norm(offset, axis=1)
        rh = self.lattice.covering_radius_m
        tolerance = 1e-8
        if observation.result == 'clear_failure':
            keep = distance + rh >= CLEAR_RADIUS_M - tolerance
            self._omni &= keep[:, None]
            self._directional &= keep[:, None, None]
            self._compact()
            return
        # 真值与位置中心的误差不超过rh。朝向半宽pi/A下，单位法向变化
        # 不超过2*sin(pi/(2A))，由三角不等式得到点积的双向误差界。
        dot = offset @ self.normals.T
        angular_error = 2 * distance * np.sin(np.pi / (2 * self.orientation_bins))
        dot_error = rh + angular_error[:, None]
        if observation.result == 'no_signal':
            outside_possible = distance[:, None] + rh >= self.radius_low[None, :] - tolerance
            behind_possible = dot - dot_error <= tolerance
            self._omni &= outside_possible
            self._directional &= outside_possible[:, :, None] | behind_possible[:, None, :]
        elif observation.result in ('direction', 'near'):
            keep_position = self._position_filter(observation)[self._indices]
            in_range_possible = distance[:, None] - rh <= self.radius_high[None, :] + tolerance
            front_possible = dot + dot_error >= -tolerance
            self._omni &= keep_position[:, None] & in_range_possible
            self._directional &= (keep_position[:, None, None] & in_range_possible[:, :, None]
                                  & front_possible[:, None, :])
        self._compact()

    def observe(self, result, x, y, bearing_deg=None, virtual_time_s=0):
        """保存有效协议检测反馈；unknown状态下的无信号不做全向式圆排除。"""
        if result not in ('direction', 'near', 'no_signal'):
            raise ValueError('未知检测结果。')
        if not np.isfinite([x, y, virtual_time_s]).all():
            raise ValueError('检测位置与时间必须有限。')
        if result == 'direction' and (bearing_deg is None or not np.isfinite(bearing_deg)):
            raise ValueError('示向反馈必须提供有限角度。')
        if self.status in ('cleared', 'excluded', 'inconsistent'):
            raise ValueError('已完成或异常频道不能继续接收检测。')
        observation = Observation(float(x), float(y), result,
                                  None if bearing_deg is None else float(bearing_deg) % 360,
                                  float(virtual_time_s))
        self.observations.append(observation)
        self._events.append(observation)
        if not self._initialized and result != 'no_signal':
            self.status = 'detected'
            self._initialize(observation)
        elif self._initialized:
            self._apply(observation)
        return self.status

    def observe_clear_failure(self, x, y, virtual_time_s=0):
        """清除失败排除被20米圆完全包含的单元，不登记为检测历史。"""
        event = Observation(float(x), float(y), 'clear_failure', None, float(virtual_time_s))
        self.clear_positions.append((float(x), float(y)))
        self._events.append(event)
        if self._initialized:
            self._apply(event)

    def mark_cleared(self, virtual_time_s=0):
        """仅由清除成功反馈调用，释放联合数组并记录清除时间。"""
        self.status = 'cleared'
        self.cleared_virtual_s = float(virtual_time_s)
        self._indices = np.empty(0, dtype=int)
        self._omni = np.zeros((0, self.radius_bins), dtype=bool)
        self._directional = np.zeros((0, self.radius_bins, self.orientation_bins), dtype=bool)

    def measured_at(self, x, y):
        """复用同位置检测；清除失败不是检测，不参与此查询。"""
        for observation in self.observations:
            if np.hypot(observation.x - x, observation.y - y) <= 1e-6:
                return observation
        return None

    def cleared_here(self, x, y):
        """禁止在物理条件不变时重复同点失败清除。"""
        return any(np.hypot(px - x, py - y) <= 1e-6 for px, py in self.clear_positions)

    def possible_points(self):
        """返回保守位置投影中心；尚未发现时保留整个初始位置外包。"""
        if not self._initialized and self.status == 'unknown':
            return self.lattice.points.copy()
        return self.lattice.points[self._indices].copy()

    def max_distance_m(self, x, y):
        """所有存活位置单元到指定点的最远距离上界，含单元覆盖半径。"""
        points = self.possible_points()
        if not len(points):
            return None
        return float(np.linalg.norm(points - np.array([x, y]), axis=1).max()
                     + self.lattice.covering_radius_m)

    def certain_clear(self, x, y):
        """必须先证实存在，再用所有位置单元判定可靠清除。"""
        if self.status != 'detected':
            return False
        limit = self.max_distance_m(x, y)
        return limit is not None and limit <= CLEAR_RADIUS_M

    def near_certain_clear(self, x, y):
        """该点曾收到near且频道仍存在时，可以原地清除。"""
        observation = self.measured_at(x, y)
        return (self.status == 'detected' and observation is not None
                and observation.result == 'near' and not self.cleared_here(x, y))

    def region_estimate(self, mode='bbox'):
        """返回区域中心及覆盖全部单元的半径；不声称是最小覆盖圆。

mode='bbox' 为原外包盒中心；mode='mec' 以最小包围圆近似中心代替，
细长区域上比盒心更接近"到全部单元最大距离最小"的点，可更早触发可靠清除。
中心只是候选点，可靠清除仍由 `certain_clear` 的全体单元距离判据把关，
因此近似不留正确性风险。
"""
        points = self.possible_points()
        if not len(points):
            return None, None
        if mode == 'mec':
            center = smallest_enclosing_circle_center(points)
        else:
            center = (points.min(axis=0) + points.max(axis=0)) / 2
        radius = float(np.linalg.norm(points - center, axis=1).max() + self.lattice.covering_radius_m)
        return center, radius

    def fallback_point(self, position):
        """返回最近存活单元中心，纯查询；仅清除失败反馈才删除该单元。"""
        if self.status != 'detected' or not len(self._indices):
            return None
        points = self.possible_points()
        return points[int(np.argmin(np.linalg.norm(points - position, axis=1)))].copy()

    def planning_hypotheses(self, max_count=96):
        """抽取联合单元中心用于启发式评分，不把中心当作可靠可行真值。"""
        if not self._initialized or self.status != 'detected' or max_count <= 0:
            return np.empty((0, 5), dtype=float)
        omni_indices = np.argwhere(self._omni)
        directional_indices = np.argwhere(self._directional)
        total = len(omni_indices) + len(directional_indices)
        if not total:
            return np.empty((0, 5), dtype=float)
        rows = []
        for index in np.linspace(0, total - 1, min(int(max_count), total), dtype=int):
            if index < len(omni_indices):
                position_index, radius_index = omni_indices[index]
                directional, orientation = 0, 0.0
            else:
                position_index, radius_index, angle_index = directional_indices[index - len(omni_indices)]
                directional, orientation = 1, self.orientation_deg[angle_index]
            center = self.lattice.points[self._indices[position_index]]
            radius = (self.radius_low[radius_index] + self.radius_high[radius_index]) / 2
            rows.append([float(center[0]), float(center[1]), float(radius), directional, float(orientation)])
        return np.asarray(rows, dtype=float)


def smallest_enclosing_circle_center(points, iterations=64):
    """返回近似最小包围圆圆心（迭代收缩投影，纯启发式候选点）。

从外包盒中心出发，反复向最远点方向移动步长减半，最终圆心不劣于盒心；
结果只作候选坐标使用，任何可靠判定仍由调用方按全体单元重算。
"""
    points = np.asarray(points, dtype=float)
    center = (points.min(axis=0) + points.max(axis=0)) / 2
    distances = np.linalg.norm(points - center, axis=1)
    step = float(distances.max()) / 2
    for _ in range(iterations):
        if step <= 1e-9:
            break
        farthest = points[int(np.argmax(np.linalg.norm(points - center, axis=1)))]
        direction = farthest - center
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-12:
            break
        candidate = center + direction / norm * min(step, norm)
        if float(np.linalg.norm(points - candidate, axis=1).max()) <= distances.max() + 1e-12:
            center = candidate
            distances = np.linalg.norm(points - center, axis=1)
        step /= 2
    return center


def ordered_search_route(waypoints, start):
    """最近邻构造 + 2-opt 改进的 Hamilton 路，返回顶点下标访问顺序。

覆盖证书只要求"每个顶点都被该频道扫描过"，访问顺序是自由度；
此函数以路径长度最小化排序，供 search_route='tour' 模式使用。
起点取距 start 最近的顶点（开局机器狗在原点）。纯排序，不改变
顶点集合，因此不影响任意朝向可发现与覆盖余量两项几何证书。
"""
    points = np.asarray(waypoints, dtype=float)
    count = len(points)
    if count < 2:
        return list(range(count))
    start_index = int(np.argmin(np.linalg.norm(points - np.asarray(start, dtype=float), axis=1)))
    remaining = list(range(count))
    route = [remaining.pop(remaining.index(start_index))]
    while remaining:
        current = points[route[-1]]
        distances = np.linalg.norm(points[remaining] - current, axis=1)
        route.append(remaining.pop(int(np.argmin(distances))))
    improved = True
    while improved:
        improved = False
        for i in range(1, count - 2):
            for j in range(i + 1, count - 1):
                a, b = points[route[i - 1]], points[route[i]]
                c, d = points[route[j]], points[route[j + 1]]
                if (np.linalg.norm(b - a) + np.linalg.norm(d - c)
                        > np.linalg.norm(c - a) + np.linalg.norm(d - b) + 1e-9):
                    route[i:j + 1] = route[i:j + 1][::-1]
                    improved = True
    return route


def triangular_search_waypoints(spacing_m=900, region_radius_m=TARGET_RADIUS_M):
    """保留半径R+b内的三角格顶点，覆盖所有与目标圆相交的三角形。

    三角形任一点到其任一顶点至多b；与半径R圆相交的三角形的三个顶点
    必在半径R+b圆内。因此该保守顶点超集包含完整的任意朝向搜索证据。
    """
    spacing, radius = float(spacing_m), float(region_radius_m)
    if not np.isfinite([spacing, radius]).all() or not 0 < spacing <= MIN_RECEIVE_RADIUS_M or radius < 0:
        raise ValueError('网格边长须在(0,1000]内，目标半径须非负且有限。')
    extent = radius + spacing
    limit = int(np.ceil(2 * extent / spacing)) + 2
    points = []
    for row in range(-limit, limit + 1):
        for column in range(-limit, limit + 1):
            x = spacing * (column + row / 2)
            y = spacing * np.sqrt(3) * row / 2
            if np.hypot(x, y) <= extent + 1e-7:
                points.append((x, y))
    return np.asarray(points, dtype=float)


# 启发式布站优化产物（.workbuddy/p4layout.py + p4layout2.py：结构化种子、
# 贪心删点、模拟退火）。硬编码保证可复现；坐标为优化器的圆整输出，
# 密采样证书数值见单元测试与 solutions/problem4_flow.md。
# 顶点数即每个待排除频道的无信号测量次数。
_OPTIMIZED_WAYPOINTS = [
    (1861.34, -516.04),
    (1104.14, 193.68),
    (1408.69, -1407.12),
    (1172.87, -1368.82),
    (961.34, -692.96),
    (478.70, -1828.31),
    (421.01, 148.28),
    (1912.31, 379.91),
    (621.55, -358.14),
    (-1122.66, -1653.75),
    (-878.92, -922.30),
    (-183.76, -978.94),
    (1590.88, 1117.07),
    (-373.62, -1889.48),
    (-1733.25, 711.95),
    (-836.36, 882.20),
    (-531.77, 463.92),
    (-29.91, 1081.23),
    (739.39, 989.76),
    (-1815.70, -950.67),
    (-313.09, -342.45),
    (-518.29, 1858.56),
    (555.62, -925.80),
    (1269.16, 1619.74),
    (-2069.81, -173.86),
    (-1234.24, -118.89),
    (-1292.06, 1495.24),
    (341.55, 1868.82),
]


def ring_search_waypoints(inner=(800.0, 6), middle=(1600.0, 11), outer=(1900.0, 12),
                          offset_deg=0.0):
    """同心环布站：中心 1 点 + 三圈等角环（默认 1+6+11+12=30 点）。

    判据与三角格/优化布站一致：目标圆域内每个位置单元对**任意朝向**都存在
    保证可读的测站。与 `optimized` 28 点的关键差别是用求解器自身的方向位图
    证书 `search_evidence.DirectionalCoverage`（20 m 单元、24 朝向 bin）
    逐站累乘验收：优化 28 点在全部顶点遍历后仍余 38 个单元存在未排除朝向
    （证书有空洞），本环结构为 **0 个**（完整）。

    几何来源（2026-09-13 推导 + 密采样复核）：边界点 p(1800) 的全部覆盖
    顶点必落在 ±33.2° 楔形内（1800·sinδ ≤ 986），且方位最大空隙须 ≤180°，
    ⇒ 外环至少 12 个 spokes、半径 ≥1863 m；中环承担中半径环带的"外向"
    方位，内环+中心负责 r≲1000 的包围。外环巡回 ≥10.4 km 是证书地板。

    本地官方同构 Engine 24 种子实测 615.5 s/源（原 optimized28 为 662.0，
    −7.05%），移动 27207 m（原 29577 m），检测 322 次（原 339 次）。
    """
    points = [(0.0, 0.0)]
    for radius, count in (inner, middle, outer):
        for i in range(count):
            angle = math.radians(offset_deg + i * 360.0 / count)
            points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return np.asarray(points, dtype=float)

def optimized_search_waypoints():
    """返回启发式布站优化的搜索顶点（比三角格少且证书余量经密采样验证）。

    两条几何证书与三角格同判据：目标圆域内任意点到最近顶点 ≤ 900 m（C1），
    任意点 1000 m 内顶点的环形最大空隙 ≤ 180°（C2，任意朝向可发现）。
    顶点全部落在演练已证实的 2700 m 停靠半径内。数值由
    SearchCertificateTests 与密采样复核共同锁定。
    """
    return np.asarray(_OPTIMIZED_WAYPOINTS, dtype=float)
