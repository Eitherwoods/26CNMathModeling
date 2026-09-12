# -*- coding: utf-8 -*-
"""问题四五项决策：多方向搜索、联合反馈追踪及有限覆盖清除。

连续集合的保证来自保守单元外包；有限假设的最坏反馈评分仅用于选点。
本模块导入时不发请求。真实会话的 enter/exit 统一由 run_strategy 管理。

**优化目标是虚拟时间**（机器狗在虚拟世界里的耗时），不是本地计算时间。
虚拟时间口径：移动=距离/5 m·s⁻¹，每次检测 5 s，频道切换 +1 s，光学定位与清除 5 s。
12 源案例实测构成：移动 36227 s（90.5%）/ 检测 3142 s / 切换 579 s / 清除 60 s，
合计 40008 s；因此任何改动都要先问"它减少行进了吗"，本地耗时不是评价口径。
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass

import numpy as np

from .base_models import (open_route_order, adaptive_directions, bounded_candidates,
                          bearing_intersection, directional_coverage_complete)

from . import config as settings
from .problem3_model import Lattice
from .problem3_solution import summarize as summarize_base
from .problem4_model import (Problem4Knowledge, optimized_search_waypoints,
                             ring_search_waypoints,
                             ordered_search_route, triangular_search_waypoints)
from .strategy import BudgetReached


@dataclass(frozen=True)
class Problem4Config:
    """问题四待调参数；位置单元必须能被一个20米清除圆覆盖。"""

    lattice_spacing_m: float = settings.PROBLEM4_LATTICE_SPACING_M
    search_spacing_m: float = settings.PROBLEM4_SEARCH_SPACING_M
    search_layout: str = settings.PROBLEM4_SEARCH_LAYOUT
    # 同心环布站（search_layout='rings'）的 (半径, 环上点数) 与整体角度偏移。
    ring_inner_m: tuple = settings.PROBLEM4_RING_INNER
    ring_middle_m: tuple = settings.PROBLEM4_RING_MIDDLE
    ring_outer_m: tuple = settings.PROBLEM4_RING_OUTER
    ring_offset_deg: float = settings.PROBLEM4_RING_OFFSET_DEG
    orientation_bins: int = settings.PROBLEM4_ORIENTATION_BINS
    radius_bins: int = settings.PROBLEM4_RADIUS_BINS
    hypothesis_limit: int = settings.PROBLEM4_HYPOTHESIS_LIMIT
    search_interval: int = settings.PROBLEM4_SEARCH_INTERVAL
    finish_detected_before_search: bool = settings.PROBLEM4_FINISH_DETECTED_BEFORE_SEARCH
    tracking_limit: int = settings.PROBLEM4_TRACKING_LIMIT
    fairness_age_rounds: int = settings.PROBLEM4_FAIRNESS_AGE_ROUNDS
    max_rounds: int = settings.PROBLEM4_MAX_ROUNDS
    info_threshold: float = settings.PROBLEM4_INFO_THRESHOLD
    exit_reserve_s: float = settings.PROBLEM4_EXIT_RESERVE_S
    step_lengths_m: tuple = settings.PROBLEM4_STEP_LENGTHS_M
    search_route: str = settings.PROBLEM4_SEARCH_ROUTE
    # 动态 2-opt 搜索巡回（2026-09-12 采纳，11 案例合计 −7.7%）：每次进入搜索
    # 决策时，对"尚有未覆盖频道的顶点"从当前位置重排最优开放路并执行首站，
    # 替代固定的"最近未覆盖顶点"贪心。静态 2-opt 比 greedy 短 14.3%（28 顶点
    # 23825→20841 m），动态重排在追踪中断后能自然回到最优续行方向。
    # 只改访问顺序，不改顶点集合，覆盖证书与完成判据不受影响。
    rolling_search_route: bool = True
    rolling_route_min_clears: int = 0
    search_route_starts: int = 1
    adaptive_initial_direction: bool = False
    tracking_path_limit_m: float = settings.PROBLEM4_TRACKING_PATH_LIMIT_M
    allocation_by_region: bool = False
    allocation_by_plan: bool = settings.PROBLEM4_ALLOCATION_BY_PLAN
    opportunistic_search: bool = False
    tracking_travel_weight: float = 0.0
    adaptive_search_evidence: bool = False
    continuous_search_evidence: bool = False
    # 与 q3 共用示向交会候选；试探失败只按实际清除反馈收缩外包。
    lls_probe: bool = False
    lls_probe_min_reads: int = 2
    lls_probe_max_mask_dist_m: float = 200.0
    lls_probe_min_separation_m: float = 25.0
    fallback_center_first: bool = False
    fallback_mec_weight: float = 2.0
    # 区域中心选法：'bbox'=外包盒中心；'mec'=最小包围圆近似中心（候选点，
    # 可靠清除仍由全体单元距离判据把关）。追踪候选可附加定距接近环；
    # 后备清除停靠时可选择性顺带测量已发现频道。
    clear_center_mode: str = 'bbox'
    approach_ring_m: tuple = ()
    measure_during_fallback: bool = False
    # 自适应追踪轮数：按存活位置单元数缩放有效追踪上限。单元很少的频道
    # 后备链极短（每步约3s+短距移动），不值得再花检测轮；单元很多时多给
    # 轮数可整片收缩区域。0 表示关闭自适应。
    shared_gain_weight_s: float = 60.0
    shortlist_radius_factor: float = 1.05
    adaptive_units_low: int = settings.PROBLEM4_ADAPTIVE_UNITS_LOW
    adaptive_units_high: int = settings.PROBLEM4_ADAPTIVE_UNITS_HIGH
    adaptive_extra_rounds: int = settings.PROBLEM4_ADAPTIVE_EXTRA_ROUNDS
    # 追击候选二级评分权重：半径接近时按"移动秒数 − 权重×共享信息增益"排序；
    # 权重越大越愿意为信息增益绕路。shortlist 半径因子控制"接近最优"的容差。
    shared_gain_weight_s: float = 60.0
    shortlist_radius_factor: float = 1.05

    def __post_init__(self):
        """在任何动作之前拒绝破坏几何保证或调度活性的参数。"""
        if type(self.search_route_starts) is not int or not 1 <= self.search_route_starts <= 8:
            raise ValueError('搜索路径初始化次数必须为1到8的整数。')
        if not isinstance(self.lls_probe, bool):
            raise ValueError('lls_probe 必须为布尔值。')
        if not isinstance(self.fallback_center_first, bool):
            raise ValueError('fallback_center_first 必须为布尔值。')
        if not np.isfinite(self.fallback_mec_weight) or self.fallback_mec_weight < 0:
            raise ValueError('后备点MEC权重必须为非负有限值。')
        if (type(self.lls_probe_min_reads) is not int or self.lls_probe_min_reads < 2):
            raise ValueError('lls_probe_min_reads 必须为不小于2的整数。')
        if (not np.isfinite(self.lls_probe_max_mask_dist_m)
                or self.lls_probe_max_mask_dist_m < 0):
            raise ValueError('交会与掩码距离必须非负且有限。')
        if (not np.isfinite(self.lls_probe_min_separation_m)
                or self.lls_probe_min_separation_m < settings.CLEAR_RADIUS_M):
            raise ValueError('交会试探间距不得小于清除半径。')
        for name in ('rolling_search_route', 'adaptive_initial_direction', 'allocation_by_region', 'allocation_by_plan',
                     'opportunistic_search', 'adaptive_search_evidence', 'continuous_search_evidence'):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f'{name} 必须为布尔值。')
        if not np.isfinite(self.tracking_path_limit_m) or self.tracking_path_limit_m < 0:
            raise ValueError('追踪路径限制必须非负且有限。')
        if not np.isfinite(self.tracking_travel_weight) or self.tracking_travel_weight < 0:
            raise ValueError('追踪移动权重必须非负且有限。')
        if (isinstance(self.rolling_route_min_clears, bool)
                or not isinstance(self.rolling_route_min_clears, int)
                or not 0 <= self.rolling_route_min_clears <= settings.MAX_SOURCE_COUNT):
            raise ValueError('路线优化启动阈值必须是0到16的整数。')
        if not (0 < self.lattice_spacing_m * np.sqrt(2) / 2 < settings.CLEAR_RADIUS_M):
            raise ValueError('位置单元覆盖半径必须严格小于20米。')
        if self.search_layout not in ('triangular', 'optimized', 'rings'):
            raise ValueError("搜索布站只允许 'triangular'、'optimized' 或 'rings'。")
        if self.search_layout == 'triangular' and not 0 < self.search_spacing_m <= settings.MIN_RECEIVE_RADIUS_M:
            raise ValueError('搜索三角形边长必须在(0,1000]米内。')
        for value in (self.orientation_bins, self.radius_bins, self.hypothesis_limit,
                      self.search_interval, self.tracking_limit, self.fairness_age_rounds,
                      self.max_rounds):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError('离散规模和调度次数必须为正整数。')
        if not np.isfinite(self.info_threshold) or self.info_threshold < 0:
            raise ValueError('信息阈值必须非负且有限。')
        if not np.isfinite(self.exit_reserve_s) or self.exit_reserve_s < 0:
            raise ValueError('退出预留时间必须非负且有限。')
        if not self.step_lengths_m or any(not np.isfinite(v) or v <= 0 for v in self.step_lengths_m):
            raise ValueError('候选步长必须为正的有限值。')
        if not isinstance(self.finish_detected_before_search, bool):
            raise ValueError('连续追踪开关必须为布尔值。')
        if self.search_route not in ('greedy', 'tour'):
            raise ValueError("搜索路线只允许 'greedy' 或 'tour'。")
        if self.clear_center_mode not in ('bbox', 'mec'):
            raise ValueError("区域中心选法只允许 'bbox' 或 'mec'。")
        if any(not np.isfinite(v) or v <= 0 for v in self.approach_ring_m):
            raise ValueError('接近环距离必须为正的有限值。')
        if not isinstance(self.measure_during_fallback, bool):
            raise ValueError('后备停靠顺带测量开关必须为布尔值。')
        for name in ('adaptive_units_low', 'adaptive_units_high', 'adaptive_extra_rounds'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError('自适应追踪阈值必须为非负整数。')
        if not np.isfinite(self.shared_gain_weight_s) or self.shared_gain_weight_s < 0:
            raise ValueError('共享信息增益权重必须非负且有限。')
        if not np.isfinite(self.shortlist_radius_factor) or self.shortlist_radius_factor < 1:
            raise ValueError('shortlist 半径因子必须≥1且有限。')


@dataclass
class StopPlan:
    """前两项决策产生的拟停靠点；尚未代表实际移动。"""

    waypoint: np.ndarray
    kind: str
    channel: int | None = None
    search_index: int | None = None


def feedback_score(hypotheses, waypoint):
    """枚举样本相容反馈，返回最坏位置外接半径和最坏权重收缩率。

示向度按±1度区间的端点枚举，跨0度使用环形角差；不假定误差概率分布。
单元中心不是精确可行参数，此分数只能排序，不能用于排除或可靠清除。
"""
    if len(hypotheses) == 0:
        return float('inf'), 0.0
    offset = hypotheses[:, :2] - waypoint
    distances = np.linalg.norm(offset, axis=1)
    angles = np.deg2rad(hypotheses[:, 4])
    front = -offset[:, 0] * np.cos(angles) - offset[:, 1] * np.sin(angles) >= 0
    received = (distances <= hypotheses[:, 2]) & ((hypotheses[:, 3] == 0) | front)
    near = received & (distances <= settings.NEAR_DISTANCE_M)
    direction = received & ~near
    bearings = np.rad2deg(np.arctan2(offset[:, 1], offset[:, 0])) % 360
    groups = [~received, near]
    error = settings.ANGLE_ERROR_DEG
    for angle in np.concatenate((bearings[direction] - error, bearings[direction] + error)):
        difference = np.abs((bearings - angle + 180) % 360 - 180)
        groups.append(direction & (difference <= error + 1e-9))
    worst_radius, worst_count = 0.0, 0
    for group in groups:
        points = hypotheses[group, :2]
        if not len(points):
            continue
        center = (points.min(axis=0) + points.max(axis=0)) / 2
        worst_radius = max(worst_radius, float(np.linalg.norm(points - center, axis=1).max()))
        worst_count = max(worst_count, len(points))
    return worst_radius, 1.0 - worst_count / len(hypotheses)


class Problem4Strategy:
    """按频道保存覆盖证据，交替推进有限搜索与有限清除的在线策略。"""

    def __init__(self, context, config=None):
        """初始化几何与记录；只读取公开协议状态，不接触案例真值。"""
        if context.problem != 4:
            raise ValueError('问题四策略仅适用于problem=4。')
        self.context = context
        self.config = config or Problem4Config()
        self.started_at = time.monotonic()
        self.lattice = Lattice.build(self.config.lattice_spacing_m)
        self.waypoints = (triangular_search_waypoints(self.config.search_spacing_m)
                          if self.config.search_layout == 'triangular'
                          else ring_search_waypoints(self.config.ring_inner_m,
                                                     self.config.ring_middle_m,
                                                     self.config.ring_outer_m,
                                                     self.config.ring_offset_deg)
                          if self.config.search_layout == 'rings'
                          else optimized_search_waypoints())
        self.search_route_order = (ordered_search_route(self.waypoints, np.zeros(2))
                                   if self.config.search_route == 'tour' else None)
        self.channels = {c: Problem4Knowledge(c, self.lattice,
                         orientation_bins=self.config.orientation_bins,
                         radius_bins=self.config.radius_bins)
                         for c in range(1, settings.CHANNEL_COUNT + 1)}
        self.coverage = {c: set() for c in self.channels}
        self.channel_series = {c: [] for c in self.channels}
        self.track_counts = {c: 0 for c in self.channels}
        self.last_served = {c: -1 for c in self.channels}
        self.position = np.array(context.state.position, dtype=float)
        self.current_channel = int(context.state.channel)
        self.start_virtual = float(context.state.virtual_time_s)
        self.steps, self.trajectory = [], [self.position.tolist()]
        self.moved_distance_m = 0.0
        self.round = 0
        self.stop_reason = 'running'
        self.search_evidence = None
        self.search_masks = {}
        self.continuous_evidence_cache = {}
        if self.config.adaptive_search_evidence:
            from .search_evidence import DirectionalCoverage
            self.search_evidence = DirectionalCoverage(self.lattice)
            self.search_masks = {c: self.search_evidence.new_mask() for c in self.channels}

    def _limited(self):
        """同时检查实际时限、协议状态和虚拟时限，预留正常退出时间。"""
        remaining = self.context.remaining_real_duration_s
        return (self.context.should_stop()
                or (remaining is not None and remaining <= self.config.exit_reserve_s)
                or self.context.state.virtual_time_s >= self._virtual_limit())

    def _virtual_limit(self):
        """以enter反馈为准获取虚拟时间上限，兼容纯内存脚手架上下文。"""
        limit = getattr(self.context.state, 'max_virtual_duration_s', None)
        return settings.VIRTUAL_BUDGET_S if limit is None else float(limit)

    def _check_action_budget(self, point, kind, channel):
        """发令前计入本次移动和动作最坏耗时，避免动作跨越虚拟时限。"""
        travel = float(np.linalg.norm(point - self.position)) / settings.ROBOT_SPEED_MPS
        action = (settings.MEASURE_TIME_S + (channel != self.current_channel)
                  if kind == 'measure' else settings.CLEAR_FAIL_TIME_S + settings.CLEAR_TIME_S)
        if self._limited() or self.context.state.virtual_time_s + travel + action >= self._virtual_limit():
            raise BudgetReached('问题四本次动作将耗尽时间预算。')

    def _detected(self):
        """返回存在性已证实、仍待清除的频道。"""
        return [c for c, knowledge in self.channels.items() if knowledge.status == 'detected']

    def _search_plan(self):
        """选择尚欠至少一个未知频道的网格顶点，保证覆盖进度单调。

greedy 模式保持原最近邻选点；tour 模式按预排 Hamilton 路的先后次序
取第一个未覆盖顶点——顶点集合不变，覆盖证据与完成判据不受影响。
"""
        unknown = [c for c, k in self.channels.items() if k.status == 'unknown']
        candidates = [i for i in range(len(self.waypoints))
                      if any(i not in self.coverage[c] for c in unknown)]
        if not candidates:
            return None
        if (self.config.rolling_search_route and
                sum(k.status == 'cleared' for k in self.channels.values()) >= self.config.rolling_route_min_clears):
            order = open_route_order(self.waypoints[candidates], self.position,
                                     starts=self.config.search_route_starts)
            index = candidates[order[0]]
        elif self.search_route_order is not None:
            rank = {index: order for order, index in enumerate(self.search_route_order)}
            index = min(candidates, key=lambda i: rank[i])
        else:
            index = min(candidates, key=lambda i: np.linalg.norm(self.waypoints[i] - self.position))
        return StopPlan(self.waypoints[index].copy(), 'search', search_index=index)

    def _tracking_candidates(self, knowledge):
        """生成接近、换侧、历史有效点凸包及失联缩步候选。"""
        center, _ = knowledge.region_estimate(self.config.clear_center_mode)
        if center is None:
            return []
        vector = center - self.position
        length = float(np.linalg.norm(vector))
        axis = vector / length if length > 1e-9 else np.array([1.0, 0.0])
        normal = np.array([-axis[1], axis[0]])
        points = [center]
        lengths = list(self.config.step_lengths_m)
        if knowledge.observations and knowledge.observations[-1].result == 'no_signal':
            lengths += [value / 2 for value in self.config.step_lengths_m]
        for distance in lengths:
            points.extend([self.position + distance * axis,
                           center + distance * normal, center - distance * normal])
        if self.config.adaptive_initial_direction:
            directions = adaptive_directions(self.position, center, 8)
            for distance in lengths:
                points.extend(self.position + distance * direction for direction in directions)
        # 定距接近环：站在距区域中心 d 处朝向机器狗一侧，参考问题三的
        # approach_direct_m——既不贴脸也不越界，减少过近/过远的无效读数。
        for distance in self.config.approach_ring_m:
            points.append(center + distance * axis)
        positive = [np.array([o.x, o.y]) for o in knowledge.observations
                    if o.result in ('direction', 'near')]
        if len(positive) >= 2:
            points.append(np.mean(positive, axis=0))
            points.extend((positive[-1] + p) / 2 for p in positive[:-1])
        return [p for p in bounded_candidates(points, self.position, self.config.tracking_path_limit_m)
                if knowledge.measured_at(*p) is None]

    def _tracking_limit_for(self, knowledge):
        """按存活单元数缩放有效追踪上限；关闭自适应时返回配置值。"""
        limit = self.config.tracking_limit
        if self.config.adaptive_units_low or self.config.adaptive_units_high:
            units = len(knowledge.possible_points())
            if self.config.adaptive_units_low and units <= self.config.adaptive_units_low:
                limit = 0
            elif (self.config.adaptive_units_high
                  and units >= self.config.adaptive_units_high):
                limit = self.config.tracking_limit + self.config.adaptive_extra_rounds
        return limit

    def _lls_probe_point(self, knowledge):
        """生成可辨识交会点，经目标域、失败间距和保守位置投影护栏筛选。"""
        if not self.config.lls_probe or knowledge.status != 'detected':
            return None
        readings = [(o.x, o.y, o.bearing_deg) for o in knowledge.observations
                    if o.result == 'direction' and o.bearing_deg is not None]
        point = bearing_intersection(readings, self.config.lls_probe_min_reads)
        if point is None or np.linalg.norm(point) > settings.TARGET_RADIUS_M + 250.0:
            return None
        if any(np.linalg.norm(point - np.asarray(old)) < self.config.lls_probe_min_separation_m
               for old in knowledge.clear_positions):
            return None
        points = knowledge.possible_points()
        if (not len(points) or np.linalg.norm(points - point, axis=1).min()
                > self.config.lls_probe_max_mask_dist_m):
            return None
        return point

    def _fallback_point(self, knowledge):
        """在存活格点中兼顾当前位置和MEC，生成短而连续的后备清除链。"""
        reference = self.position
        if self.config.fallback_center_first and not knowledge.clear_positions:
            center, _ = knowledge.region_estimate(self.config.clear_center_mode)
            if center is not None:
                reference = center
        points = knowledge.possible_points()
        if not len(points):
            return None
        center, _ = knowledge.region_estimate('mec')
        if center is None or self.config.fallback_mec_weight <= 0:
            return knowledge.fallback_point(reference)
        score = (np.linalg.norm(points - reference, axis=1)
                 + self.config.fallback_mec_weight * np.linalg.norm(points - center, axis=1))
        return points[int(np.argmin(score))].copy()

    def _tracking_plan(self, channel):
        """先限定有限追踪轮数，再转入覆盖清除，防止失联后无限往返。"""
        knowledge = self.channels[channel]
        center, _ = knowledge.region_estimate(self.config.clear_center_mode)
        if center is not None and knowledge.certain_clear(*center) and not knowledge.cleared_here(*center):
            return StopPlan(center, 'reliable_clear', channel)
        probe = self._lls_probe_point(knowledge)
        if probe is not None:
            return StopPlan(probe, 'lls_probe', channel)
        if self.track_counts[channel] >= self._tracking_limit_for(knowledge):
            point = self._fallback_point(knowledge)
            return None if point is None else StopPlan(point, 'fallback_clear', channel)
        candidates = self._tracking_candidates(knowledge)
        if not candidates:
            point = self._fallback_point(knowledge)
            return None if point is None else StopPlan(point, 'fallback_clear', channel)
        hypotheses = knowledge.planning_hypotheses(self.config.hypothesis_limit)
        scores = [(feedback_score(hypotheses, p), p) for p in candidates]
        if self.config.tracking_travel_weight:
            _, point = min(scores, key=lambda item: item[0][0] + self.config.tracking_travel_weight
                           * float(np.linalg.norm(item[1] - self.position)))
            return StopPlan(point, 'track', channel)
        best_radius = min(score[0] for score, _ in scores)
        shortlisted = [(score, p) for score, p in scores
                       if score[0] <= best_radius * self.config.shortlist_radius_factor + 1e-9]
        # 半径接近时，以联合信息、移动代价区分；权重不解释为成功概率。
        shared_hypotheses = [self.channels[c].planning_hypotheses(self.config.hypothesis_limit)
                             for c in self._detected() if c != channel]
        def secondary_score(item):
            """合并多个已发现频道的信息价值，移动成本仅计一次。"""
            score, point = item
            shared_gain = score[1] + sum(feedback_score(h, point)[1] for h in shared_hypotheses)
            return (np.linalg.norm(point - self.position) / settings.ROBOT_SPEED_MPS
                    - self.config.shared_gain_weight_s * shared_gain)
        _, point = min(shortlisted, key=secondary_score)
        return StopPlan(point, 'track', channel)

    def _channel_travel_m(self, channel):
        """该频道下一个追踪目标的直线距离，用作虚拟时间的第一成本项。

        优化目标是虚拟时间，其中移动约占九成（12源案例 36227/40008 s）；因此调度
        必须优先压低行进，而不是追求"每轮都换一个频道"。此处只做排序，不改变
        任何可行集或完成判据。
        """
        knowledge = self.channels[channel]
        if (self.config.allocation_by_region
                and self.track_counts[channel] >= self._tracking_limit_for(knowledge)):
            point = knowledge.fallback_point(self.position)
            return float('inf') if point is None else float(np.linalg.norm(point - self.position))
        center, _ = knowledge.region_estimate(self.config.clear_center_mode)
        if center is None:
            point = knowledge.fallback_point(self.position)
            return float('inf') if point is None else float(np.linalg.norm(point - self.position))
        return float(np.linalg.norm(center - self.position))

    def _tracking_route_extension(self, start, channel, detected):
        """估计执行当前频道后，按保守中心访问其余频道的最短开放路线。

        该量只用于频道调度的二级排序。路线中的点来自当前频道知识的
        `region_estimate`，不作为可靠清除点，也不改变任何证据和完成判据。
        每次决策都会重新计算，因此无信号、示向和清除失败造成的排除会立即
        反映到下一轮的路线估计中。
        """
        points = []
        for other in detected:
            if other == channel:
                continue
            center, _ = self.channels[other].region_estimate(self.config.clear_center_mode)
            if center is not None:
                points.append(np.asarray(center, dtype=float))
        if not points:
            return 0.0
        points = np.asarray(points, dtype=float)
        order = open_route_order(points, np.asarray(start, dtype=float), starts=1)
        route = np.vstack((np.asarray(start, dtype=float), points[order]))
        return float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())

    def _next_channel(self, detected):
        """优先服务行进代价最小的已发现频道，超过公平年龄的频道强制优先。

        规则："最久未服务且已超龄"的先做（保证 `tracking_limit` 与后备清除一定
        轮得到每个频道，杜绝饿死），否则取下一个追踪点最近的频道。这样同一片
        区域的多个目标会被连续处理，消掉原"按频道轮转"造成的跨场往返。
        """
        aged = [c for c in detected
                if self.round - self.last_served[c] >= self.config.fairness_age_rounds]
        if aged:
            return min(aged, key=lambda c: (self.last_served[c], c))
        return min(detected, key=lambda c: (self._channel_travel_m(c), self.last_served[c], c))

    def decide_direction(self):
        """第一项决策输出方向，并优先连续处理已发现目标。

        单个目标最多经历有限次追踪，随后进入每次均删除位置单元的后备清除，
        所以连续处理必然结束；目标处理完毕后恢复三角网格覆盖，无需周期性跨区折返。
        """
        search = self._search_plan()
        detected = self._detected()
        if self.round == 0:
            plan = StopPlan(self.position.copy(), 'initial')
        elif (search is not None and
              (not detected or (not self.config.finish_detected_before_search
                                and self.round % self.config.search_interval == 0))):
            plan = search
        elif detected:
            if self.config.allocation_by_plan:
                aged = [c for c in detected
                        if self.round - self.last_served[c] >= self.config.fairness_age_rounds]
                eligible = [self._next_channel(detected)] if aged else detected
                plans = [self._tracking_plan(c) for c in eligible]
                plans = [p for p in plans if p is not None]
                def planned_seconds(candidate):
                    """与 q3 同口径：实际移动时间加检测后的剩余定位规模。"""
                    seconds = float(np.linalg.norm(candidate.waypoint - self.position)) / settings.ROBOT_SPEED_MPS
                    if candidate.kind == 'track':
                        hypotheses = self.channels[candidate.channel].planning_hypotheses(self.config.hypothesis_limit)
                        seconds += feedback_score(hypotheses, candidate.waypoint)[0] / settings.ROBOT_SPEED_MPS + 60.
                    # 将当前候选接入其余已发现频道的开放路线，避免即时最优
                    # 把机器狗带到路线末端后再跨区折返。
                    seconds += self._tracking_route_extension(
                        candidate.waypoint, candidate.channel, detected
                    ) / settings.ROBOT_SPEED_MPS
                    return seconds, candidate.channel
                plan = min(plans, key=planned_seconds) if plans else None
            else:
                plan = self._tracking_plan(self._next_channel(detected))
        else:
            plan = search
        if plan is None:
            return None, None
        vector = plan.waypoint - self.position
        length = float(np.linalg.norm(vector))
        return (vector / length if length > 1e-9 else np.zeros(2)), plan

    def decide_distance(self, direction, plan):
        """第二项沿已选方向比较停靠长度，必要覆盖与后备清除保留精确端点。"""
        if plan.kind != 'track':
            return plan
        length = float(np.linalg.norm(plan.waypoint - self.position))
        knowledge = self.channels[plan.channel]
        candidates = [self.position + direction * value for value in
                      sorted(set([length] + [v for v in self.config.step_lengths_m if v < length]))]
        candidates = [p for p in candidates if knowledge.measured_at(*p) is None]
        if not candidates:
            return plan
        hypotheses = knowledge.planning_hypotheses(self.config.hypothesis_limit)
        point = min(candidates, key=lambda p: (feedback_score(hypotheses, p)[0]
                                               + self.config.tracking_travel_weight * float(np.linalg.norm(p - self.position)),
                                               float(np.linalg.norm(p - self.position))))
        return StopPlan(point, plan.kind, plan.channel)

    def _commit_action(self, point, channel, kind, response):
        """仅accepted=true后登记真实移动和动作；清除不会修改测向频道。"""
        if response.get('accepted') is not True:
            raise RuntimeError('动作未获确认，禁止更新位置或覆盖证据。')
        self.moved_distance_m += float(np.linalg.norm(point - self.position))
        self.position = np.asarray(point, dtype=float).copy()
        self.trajectory.append(self.position.tolist())
        if kind == 'measure':
            self.current_channel = channel
        self.steps.append({'kind': kind, 'channel': channel, 'x': float(point[0]),
                           'y': float(point[1]), 'result': response[f'{kind}_result'],
                           'virtual_time_s': float(response['virtual_time_s'])})

    def execute_measures(self, plan):
        """第三项逐频道扫描并即时更新；本阶段不插入任何清除。"""
        acted = False
        order = sorted(self.channels, key=lambda c: (c != self.current_channel, c))
        for channel in order:
            if self._limited():
                break
            knowledge = self.channels[channel]
            point = plan.waypoint
            if knowledge.status not in ('unknown', 'detected') or knowledge.measured_at(*point) is not None:
                continue
            if knowledge.status == 'detected' and knowledge.certain_clear(*point):
                continue
            necessary = (plan.kind == 'initial' or
                         (plan.kind == 'search' and knowledge.status == 'unknown') or
                         (plan.kind == 'track' and channel == plan.channel))
            if (self.config.opportunistic_search and knowledge.status == 'unknown'
                    and plan.kind == 'track'):
                # 已支付移动代价，只有与历史测点相隔足够远才安排未知频道检测。
                necessary = all(np.linalg.norm(point - np.array([o.x, o.y])) >= 600.0
                                for o in knowledge.observations)
            if not necessary:
                if knowledge.status != 'detected':
                    continue
                if plan.kind == 'fallback_clear' and not self.config.measure_during_fallback:
                    continue
                _, gain = feedback_score(knowledge.planning_hypotheses(self.config.hypothesis_limit), point)
                cost = settings.MEASURE_TIME_S + (channel != self.current_channel)
                if gain / cost < self.config.info_threshold:
                    continue
            self._check_action_budget(point, 'measure', channel)
            response = self.context.measure(float(point[0]), float(point[1]), channel)
            self._commit_action(point, channel, 'measure', response)
            knowledge.observe(response['measure_result'], *point, bearing_deg=response.get('svd_deg'),
                              virtual_time_s=response['virtual_time_s'])
            if self.search_evidence is not None and knowledge.status == 'unknown':
                self.search_masks[channel] = self.search_evidence.update(self.search_masks[channel], point)
                if not self.search_masks[channel].any():
                    knowledge.status = 'excluded'
            series = self.channel_series[channel]
            limit = knowledge.max_distance_m(*point)
            series.append({'measure_count': len(series) + 1,
                           'limit_m': None if limit is None else float(limit),
                           'result': response['measure_result']})
            for index in np.flatnonzero(np.linalg.norm(self.waypoints - point, axis=1) <= 1e-7):
                self.coverage[channel].add(int(index))
            acted = True
        return acted

    def execute_clears(self, plan):
        """第四项使用扫描后的最新位置外包；无信号不否决光学清除。"""
        acted = False
        point = plan.waypoint
        for channel in self._detected():
            if self._limited():
                break
            knowledge = self.channels[channel]
            if knowledge.cleared_here(*point):
                continue
            reliable = knowledge.certain_clear(*point) or knowledge.near_certain_clear(*point)
            fallback = plan.kind in ('fallback_clear', 'lls_probe') and plan.channel == channel
            if not (reliable or fallback):
                continue
            self._check_action_budget(point, 'clear', channel)
            response = self.context.clear(float(point[0]), float(point[1]), channel)
            self._commit_action(point, channel, 'clear', response)
            if response['clear_result'] == 'success':
                knowledge.mark_cleared(response['virtual_time_s'])
            else:
                knowledge.observe_clear_failure(*point, virtual_time_s=response['virtual_time_s'])
            acted = True
        return acted

    def finished(self):
        """第五项只认可完整覆盖或清除数上界，异常空集不作为完成证据。"""
        for channel, knowledge in self.channels.items():
            if knowledge.status == 'unknown' and len(self.coverage[channel]) == len(self.waypoints):
                knowledge.status = 'excluded'
            if self.config.continuous_search_evidence and knowledge.status == 'unknown':
                stations = tuple(sorted({(o.x, o.y) for o in knowledge.observations
                                         if o.result == 'no_signal'}))
                if stations not in self.continuous_evidence_cache:
                    self.continuous_evidence_cache[stations] = directional_coverage_complete(
                        self.lattice.points, self.lattice.covering_radius_m, stations,
                        settings.MIN_RECEIVE_RADIUS_M)
                if self.continuous_evidence_cache[stations]:
                    knowledge.status = 'excluded'
        if any(k.status == 'inconsistent' for k in self.channels.values()):
            return 'model_inconsistent'
        if sum(k.status == 'cleared' for k in self.channels.values()) == settings.MAX_SOURCE_COUNT:
            return 'cleared_limit'
        if all(k.status in ('cleared', 'excluded') for k in self.channels.values()):
            return 'all_channels_resolved'
        return None

    def run(self):
        """执行有界五项循环，所有受限终止均明确区别于任务完成。"""
        for self.round in range(self.config.max_rounds):
            reason = self.finished()
            if reason or self._limited():
                self.stop_reason = reason or 'budget_limit'
                break
            try:
                direction, plan = self.decide_direction()
                if plan is None:
                    self.stop_reason = 'no_plan'
                    break
                plan = self.decide_distance(direction, plan)
                acted = self.execute_measures(plan)
                acted = self.execute_clears(plan) or acted
                if plan.channel is not None:
                    self.last_served[plan.channel] = self.round
                    self.track_counts[plan.channel] += 1
                if not acted:
                    self.stop_reason = 'budget_limit' if self._limited() else 'no_accepted_action'
                    break
            except BudgetReached:
                self.stop_reason = 'budget_limit'
                break
        else:
            self.stop_reason = self.finished() or 'round_limit'
        return self.result()

    def result(self):
        """输出可复核轨迹、逐频道覆盖进度及模型异常，不编造未知总数。"""
        return {'problem': 4, 'stop_reason': self.stop_reason,
                'completed': self.stop_reason in ('all_channels_resolved', 'cleared_limit'),
                'cleared_channels': [c for c, k in self.channels.items() if k.status == 'cleared'],
                'unresolved_channels': sorted(c for c, k in self.channels.items()
                                              if k.status in ('unknown', 'detected')),
                'inconsistent_channels': sorted(c for c, k in self.channels.items()
                                                if k.status == 'inconsistent'),
                'start_virtual_time_s': self.start_virtual,
                'end_virtual_time_s': float(self.context.state.virtual_time_s),
                'real_elapsed_s': time.monotonic() - self.started_at,
                'moved_distance_m': self.moved_distance_m, 'steps': self.steps,
                'trajectory': self.trajectory, 'channel_series': self.channel_series,
                'search_waypoints': len(self.waypoints), 'config': asdict(self.config),
                'planner_errors': int(self.stop_reason in ('model_inconsistent', 'no_plan', 'no_accepted_action')),
                'channels': {c: {'status': k.status, 'type': k.type_status,
                                  'coverage_done': len(self.coverage[c]),
                                  'coverage_required': len(self.waypoints)}
                             for c, k in self.channels.items()}}


def summarize(record, true_total=None):
    """复用题目统计口径，并单列是否已证明完成。"""
    if true_total is not None and (isinstance(true_total, bool) or not isinstance(true_total, int)
                                   or not len(record['cleared_channels']) <= true_total <= settings.CHANNEL_COUNT
                                   or true_total <= 0):
        raise ValueError('真实总数须为正整数，且不能小于已清除数或超过20。')
    steps = record['steps']
    return {**summarize_base(record, true_total), 'completed': record['completed'],
            'no_signal_count': sum(s['kind'] == 'measure' and s['result'] == 'no_signal' for s in steps),
            'clear_failure_count': sum(s['kind'] == 'clear' and s['result'] == 'no_target_in_range'
                                       for s in steps)}


def run_mission(context, config=None):
    """供测试脚手架和协议入口注入上下文；不会读取隐藏案例数据。"""
    return Problem4Strategy(context, config).run()


def is_offline_run(context):
    """判断本次会话是否跑在本地桩上：离线桩的记录是自检产物，不是演练成绩。"""
    from .offline_stub import OfflineStub
    return isinstance(getattr(context.client, 'transport', None), OfflineStub)


def solve(context):
    """运行器策略入口；仅在被明确调用时执行会话与保存问题四记录。

    记录目录按 `output/README.md` 的约定二分：**离线自检进 `output/protocol`**，
    在线（模拟器演练/正式）进 `output/Problem4`，文件名以 `offline-` 前缀进一步区分。
    """
    record = run_mission(context)
    summary = summarize(record)
    offline = is_offline_run(context)
    try:
        target_dir = settings.record_dir_for(context.problem, offline=offline)
        target_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d-%H%M%S')
        prefix = 'offline-' if offline else ''
        target = target_dir / f'{prefix}mission_p{context.problem}_{stamp}.json'
        target.write_text(json.dumps({'summary': summary, 'record': record},
                                    ensure_ascii=False, indent=2), encoding='utf-8')
        summary['record_path'] = str(target)
    except OSError as exc:
        summary['record_path'] = None
        summary['record_error'] = str(exc)
    return summary


def build_scenario(sources=12, seed=1, directional=None):
    """构造本地自检案例：directional 为 None 时取一半定向源，0 表示全部全向。"""
    from .scenario import random_scenario
    from .scenario_p4 import mixed_scenario
    count = int(sources)
    if count < 2 or directional == 0:
        return random_scenario(seed=seed, n_sources=count)
    share = count // 2 if directional is None or directional < 0 else int(directional)
    return mixed_scenario(seed=seed, n_sources=count,
                          n_directional=max(1, min(share, count - 1)))


def main(argv=None):
    """问题四入口：给记录文件则只汇总；否则用本地桩跑一局自检（不连接官方模拟器）。"""
    from pathlib import Path

    parser = argparse.ArgumentParser(description='问题四离线自检与记录汇总入口。')
    parser.add_argument('record', nargs='?', default=None,
                        help='已有任务记录 JSON；省略则在本地桩上跑一局')
    parser.add_argument('--true-total', type=int, help='真实干扰源总数；未知时不填')
    parser.add_argument('--sources', type=int, default=12, help='自检案例干扰源个数')
    parser.add_argument('--seed', type=int, default=1, help='自检案例随机种子')
    parser.add_argument('--directional', type=int, default=None,
                        help='定向源个数；默认取一半，0 表示全部全向')
    parser.add_argument('--latency', type=float, default=0.0, help='每次请求的附加真实延迟（秒）')
    parser.add_argument('--log', default='output/protocol/offline-p4.jsonl',
                        help='离线协议日志；与 run_robot --mode offline 的默认路径一致')
    parser.add_argument('--figures', action='store_true', help='输出任务图（需要 matplotlib）')
    args = parser.parse_args(argv)

    if args.record is not None:
        data = json.loads(Path(args.record).read_text(encoding='utf-8'))
        print(json.dumps(summarize(data.get('record', data), args.true_total),
                         ensure_ascii=False, indent=2))
        return None

    from .offline_stub import OfflineStub
    from .protocol import RobotClient
    from .strategy import run_strategy

    scenario = build_scenario(args.sources, args.seed, args.directional)
    stub = OfflineStub(sources=scenario.sources, latency_s=args.latency, location_error_deg=0.9)
    client = RobotClient('offline-team', stub, log_path=args.log)
    result = run_strategy(client, solve, problem=4)
    print(json.dumps(result['algorithm_result'], ensure_ascii=False, indent=2))
    if args.figures:
        from .problem4_plotting import plot_mission_from_path
        plot_mission_from_path(result['algorithm_result'].get('record_path'), scenario=scenario)
    return result


if __name__ == '__main__':
    main()
