# -*- coding: utf-8 -*-
"""问题三：全向干扰源的自动搜索定位与清除策略。

五项决策与 `solutions/modeling-strategy.md` 第 3 章一一对应
----------------------------------------------------------
决策1+2 往哪走、走多远：在"追某个已发现目标"与"去某个覆盖增益更大的停靠点"
        两类选项之间，以**每清除一个干扰源的期望耗时**为统一尺度择优。
决策3   是否扫描某频率：逐频道做状态筛选、重复检测筛选、新增约束判断与成本判断。
决策4   是否清除某频率：以第三项结束后的最新信息判断可靠清除，其次按均匀先验的
        成功概率做试探清除。
决策5   是否结束：全部频道已清除或已排除、清除数达上限、或时间预算不足时退出。

模型要点
--------
1. 题目两个统计量（清除比例、平均定位清除时间）的公共含义是"单位时间清除数"
   N/T，故所有选项按"期望秒数 / 期望清除数"比较，越小越好。
2. 严格结论（频道排除、可靠清除、成功概率）一律由 `problem3_model` 的细格网给出；
   前瞻评分在粗格网上完成，不参与任何结论。
3. 真实运行时间上限 20 分钟，而单次检测只消耗虚拟时间。策略用单调时钟估计
   "还能发多少个动作"，容量不足时自动降级为只追已发现目标，保证能正常退出。
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass, field, replace

import numpy as np

from . import config as settings
from .base_models import bearing_deg, open_route_order, adaptive_directions, bounded_candidates
from .config import (ANGLE_ERROR_DEG, CHANNEL_COUNT, CHANNEL_SWITCH_TIME_S,
                     CLEAR_RADIUS_M, CLEAR_TIME_S, MAX_RECEIVE_RADIUS_M,
                     MAX_SOURCE_COUNT, MIN_SOURCE_COUNT,
                     MEASURE_TIME_S, MIN_RECEIVE_RADIUS_M, ROBOT_SPEED_MPS,
                     TARGET_RADIUS_M)
from .problem3_model import ChannelKnowledge, Lattice, apply_direction, hex_waypoints
from .strategy import BudgetReached

TARGET_AREA_M2 = float(np.pi * TARGET_RADIUS_M ** 2)


@dataclass(frozen=True)
class Problem3Config:
    """问题三数值求解参数。"""

    lattice_spacing_m: float = 10.0
    planning_spacing_m: float = 50.0
    candidate_spacing_m: float = 1200.0
    # 六边形环半径：1400 时最坏漏检 sqrt(1800²+1400²−2·1800·1400·cos30°)
    # ≈ 914 m < 1000 m，证书仍成立；扫档显示比 1500 略优。
    candidate_radius_m: float = 1400.0
    # 候选布局：hybrid15 = 九点方格 ∪ 七点六边形（覆盖证书见 _search_plan；
    # 九点最坏漏检 848.36 m < 1000 m，并集只增不减，证书不变）。
    # 2026-09-11 多轮离线扫档：比 grid9 端到端少 ~4% 虚拟时间（更多站点
    # 使追击顺路与补证巡航的每站信息量更高）。
    candidate_layout: str = 'hybrid15'
    lookahead_directions: int = 24
    adaptive_initial_direction: bool = False
    tracking_path_limit_m: float = 0.0
    lookahead_lengths_m: tuple = (250.0, 500.0, 900.0)
    lookahead_hypotheses: int = 15
    chase_options_limit: int = 4
    allocation_by_region: bool = False
    local_lookahead_limit_m: float = 500.0
    approach_direct_m: float = 400.0
    # 追踪途中已经支付了移动代价，多测几个有价值频道通常比日后专程折返更省时。
    search_channels_per_stop: int = 3
    # 2026-09-12 联调复扫：1.5e5 比旧值 5e4 端到端再省 ~0.9%（基准 299.2 s/源；
    # 留出种子 331.8 vs 336.4），方向在两组独立案例上一致。
    search_min_gain_m2: float = 1.5e5
    # 一旦已经决定访问某个搜索停靠点，顺手完成该点全部正增益频道，避免日后折返。
    fill_search_stops: bool = True
    endgame_seconds: float = 60.0
    search_risk_premium: float = 1.25
    # 接收范围感知追击（2026-09-11 扫档采纳，8 案例再省 ~3.5%）：源的有效接收
    # 半径 a∈[1000,1500] 未知。若候选检测点到全部可能位置都超过 1000 m，则读到
    # 示向度没有保证；按保守口径把"读不到"分支的剩余逼近距离计入评分，
    # 避免在接收边界外反复空测（实测收敛链中 U 值可连续多轮停滞在 ~1000 m）。
    receive_aware_chase: bool = True
    # True 时把"无任何保证可读位置"的候选直接跳过（更激进）；False 只在
    # 超过接收半径上界（必然读不到）时跳过。惩罚倍率用于放大无信号分支代价。
    receive_aware_skip_unread: bool = False
    receive_aware_penalty_scale: float = 1.0
    # 两阶段调度（默认关闭）：只要仍有已发现未清除的频道，就不为覆盖搜索单开
    # 停靠点。问题四实测该策略大幅优于"覆盖优先"；追击途中的顺路检测
    # （search_channels_per_stop）不受影响。
    finish_detected_before_search: bool = False
    # 二步滚动前瞻（0=关闭）：对粗格网单步评分前 K 个追击候选模拟"检测后
    # 再走一步"，用 travel2+剩余2 重评。只展开小候选集，控制单次决策耗时。
    chase_rollout_topk: int = 0
    chase_rollout_lengths_m: tuple = (400.0,)
    chase_rollout_directions: int = 8
    # 无信号补证路线模拟的 2-opt 改进（默认关闭）：贪心最近站序列构造后，
    # 对访问顺序做 2-opt（站点集合与各站检测频道不变，覆盖结果不变），
    # 取改进后与原序列的较小总耗时。
    absence_route_2opt: bool = False
    rolling_search_route: bool = False
    # 承诺式补证游（默认关闭；开启时通常连同 absence_route_2opt）：把 2-opt
    # 改进后的完整路线缓存并按序执行，期间任何示向/近场读数（出现新发现）
    # 立即作废该路线。只改"估计与执行一致"，不改覆盖站点集合与证书。
    absence_tour_commit: bool = False
    # 策略形态：'mpc' 沿用逐轮重规划；'route' 启用 Route-First（巡游主导 +
    # 两站交会，见 problem3_route.py，正确性层完全共用）。
    strategy_mode: str = 'mpc'
    # Route-First 参数：巡游站点集（'hybrid15' 全部候选 / 'grid9' 证书九点）、
    # 交会不确定半径低于该值即受控偏离清除、偏离最远距离。
    route_tour_set: str = 'hybrid15'
    route_spec_radius_m: float = 80.0
    route_deviation_budget_m: float = 900.0
    # 存在已发现但交会未成熟的频道时，本轮交给 MPC 追击（实测带偏巡游，
    # 默认关闭）；route_max_nodes>0 时巡游只负责前 K 站批量发现，之后整体
    # 交给 MPC 收尾（不中途打断）。
    route_handoff_detected: bool = False
    route_max_nodes: int = 0
    # 双站交会追击（2026-09-12 联调试证伪，默认关闭，保留作消融开关）：
    # 对已有一次示向读数的频道，在其锚点侧向（垂直于示向方向）
    # `rendezvous_offsets_m` 距离上布置第二检测站候选。两示向线交会把不确定
    # 半径压到约 ε·d²/s，但实测 +3%（行进更长而信息不增）：MPC 的
    # 24 方向×15 假设前瞻已覆盖"横向换位"选项，且沿示向线的 Ray-step 才是
    # GDOP 更优的第二站几何。见 solutions/problem3_flow.md §10.2。
    rendezvous_chase: bool = False
    rendezvous_offsets_m: tuple = (600.0, 900.0, 1200.0)
    # 折返抑制（默认关闭，单位秒）：候选停靠点的方位与"上一段实际移动方位"夹角超过
    # `turn_penalty_angle_deg` 时，在其评分上加惩罚。动机：MPC 每轮贪心选点会在两个
    # 方向相反的目标之间来回切换——n12 案例实测出现 500 m 去 + 500 m 回的 A→B→A
    # 折返，且 98.7% 的移动落在被穿越 ≥3 次的 300 m 方格上。
    # 只作用于需要长距离移动的选项（`turn_penalty_min_leg_m` 以上），短距微调不受影响。
    turn_penalty_s: float = 0.0
    turn_penalty_angle_deg: float = 135.0
    turn_penalty_min_leg_m: float = 200.0
    speculative_clear_m: float = 45.0
    speculative_min_probability: float = 0.30
    real_time_reserve_s: float = 15.0
    min_action_capacity: int = 10
    max_rounds: int = 400
    stall_limit: int = 4
    # 近场候选层：在当前点周围近距离补刀，避免远距离候选导致的横穿全场。
    near_candidate_scales: tuple = (0.35, 0.7)
    near_candidate_span_m: float = 900.0
    # 覆盖兜底：残余**连通空洞**大于该面积时才专门跑一趟补齐。
    # 取值依据：单个停靠点在 r=1200 的七点布站下最坏漏检半径约 973 m，
    # 对应缺口量级 ~1e5 m² 起；取 4e5 m² 确保只在确实存在整块空洞时触发，
    # 不被零星毛刺引走（实测阈值过低会使行进反升）。
    backstop_min_hole_m2: float = 4.0e5
    backstop_max_stations: int = 12
    # 已清除数达到阈值且没有待追踪源时，动态残余空洞候选与固定格点同台评分。
    # 阈值 8 低于题面下界 10：动态方案只参与站点选择（不参与结束判定），
    # 提前激活可减少收尾阶段固定格点的跨场折返；扫档 −0.4%。
    enable_dynamic_endgame: bool = True
    dynamic_endgame_min_clears: int = 8
    # 清除数较高时，按“剩余频道均无信号”的情景估计完整补证路线，只执行首站。
    # 2026-09-11 扫档：6（题面下界 10 的 60%）比 14 端到端再省 ~3%。
    enable_route_aware_endgame: bool = True
    route_endgame_min_clears: int = 6
    # 可选停靠点回访惩罚。同站补测开启后默认关闭，避免把仍有价值的近点推迟到
    # 收尾阶段，形成跨场折返；保留参数便于关闭同站补测时做对照实验。
    revisit_radius_m: float = 400.0
    revisit_penalty_s: float = 0.0
    # 保证性示向追踪：若已发现频道长期未被服务，则强制执行一次收缩上界的追踪动作。
    enable_guaranteed_tracking: bool = False
    tracking_debt_limit_rounds: int = 10
    # 路线规划器：'greedy' 沿用最近必要站启发式；'dijkstra' 用插入启发式 + Dijkstra
    # 距离优化收尾补证路线。
    route_planner: str = 'greedy'
    # 若计划移动距离超过该阈值，则在直线路径上采样中间检测点并择优停靠；
    # 0 表示不启用。代价是 5 s 检测 + 可能 1 s 切换，必须能缩短后续行程才划算。
    # 2026-09-11 扫档：800 m 在 8 案例基准上最稳（−6% 左右）。
    intermediate_stop_gap_m: float = 800.0
    intermediate_stop_min_gain_m2: float = 2.0e5

    def __post_init__(self):
        """在构造格网或执行动作之前拒绝不稳定的搜索参数。"""
        for name in ('lattice_spacing_m', 'planning_spacing_m', 'candidate_spacing_m', 'candidate_radius_m'):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} 必须为有限正数。')
        for name in ('lookahead_directions', 'lookahead_hypotheses', 'chase_options_limit', 'max_rounds'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f'{name} 必须为正整数。')
        for name in ('rolling_search_route', 'adaptive_initial_direction', 'allocation_by_region',
                     'rendezvous_chase'):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f'{name} 必须为布尔值。')
        offsets = self.rendezvous_offsets_m
        if (not isinstance(offsets, (tuple, list)) or not offsets
                or any(isinstance(v, bool) or not np.isfinite(v) or v <= 0 for v in offsets)):
            raise ValueError('双站交会偏距必须为非空的有限正数序列。')
        if not np.isfinite(self.tracking_path_limit_m) or self.tracking_path_limit_m < 0:
            raise ValueError('追踪路径限制必须非负且有限。')
        if not self.lookahead_lengths_m or any(not np.isfinite(v) or v <= 0 for v in self.lookahead_lengths_m):
            raise ValueError('前瞻步长必须为非空的有限正数序列。')


@dataclass
class StopPlan:
    """一次停靠的决策结果：停靠点、拟检测频道与预计代价。"""

    kind: str
    waypoint: np.ndarray
    measures: list = field(default_factory=list)
    travel_m: float = 0.0
    score_s: float = float('inf')
    note: str = ''
    expect_finds: float = 0.0
    target_channel: int | None = None
    guaranteed_upper_m: float | None = None


def summarize(record, true_total=None):
    """把任务记录整理成题目表1所需的统计量。"""
    cleared = list(record['cleared_channels'])
    total_virtual = float(record['end_virtual_time_s'] - record['start_virtual_time_s'])
    count = len(cleared)
    return {
        'cleared_channels': cleared,
        'cleared_count': count,
        'true_total': true_total,
        'cleared_ratio': None if not true_total else count / true_total,
        'total_virtual_time_s': round(total_virtual, 3),
        'average_clear_time_s': None if count == 0 else round(total_virtual / count, 3),
        'program_runtime_s': round(float(record['real_elapsed_s']), 3),
        'stop_reason': record['stop_reason'],
        'unresolved_channels': list(record.get('unresolved_channels', [])),
        'inconsistent_channels': list(record.get('inconsistent_channels', [])),
        'actions': len(record['steps']),
        'moved_distance_m': round(float(record['moved_distance_m']), 3),
        'planner_errors': record['planner_errors'],
    }


class Problem3Strategy:
    """问题三策略：在给定协议上下文上执行五项决策循环。"""

    def __init__(self, context, config=None):
        self.context = context
        self.config = config or Problem3Config()
        self.fine = Lattice.build(self.config.lattice_spacing_m)
        self.coarse = Lattice.build(self.config.planning_spacing_m)
        self.candidates = self._build_candidates()
        self.channels = {channel: ChannelKnowledge(channel, self.fine, self.coarse)
                         for channel in range(1, CHANNEL_COUNT + 1)}
        self.position = tuple(float(value) for value in context.state.position)
        self.current_channel = int(context.state.channel)
        self.cleared: set = set()
        self.start_virtual_time_s = float(context.state.virtual_time_s)
        self.steps: list = []
        self.trajectory: list = [self.position]
        self.channel_series = {channel: [] for channel in self.channels}
        self.moved_distance_m = 0.0
        self.planner_errors = 0
        # 上一段"实际发生位移"的移动方位（度）；仅用于折返抑制，不参与任何严格结论。
        self.last_move_bearing_deg = None
        self.stop_reason = 'running'
        self._started_at = time.monotonic()
        self._reach_cache: dict = {}
        # channel -> {anchor: (x, y), bearing_deg: theta, upper_m: U}。
        # 该状态只记录可由示向读数严格保证的追踪上界，不参与细格网可靠结论。
        self.tracking_states: dict = {}
        self.tracking_wait_rounds = {channel: 0 for channel in self.channels}
        # 承诺式补证游：[(waypoint, ordered_channels)] 与下一站指针。
        self.absence_tour = None
        self.absence_tour_pos = 0

    def _build_candidates(self):
        """构造具有覆盖证明的搜索候选停靠点。

        默认使用九点方格方案；混合十五点与七点六边形方案保留用于消融测试。
        """
        if self.config.candidate_layout == 'hex7':
            return hex_waypoints(self.config.candidate_radius_m)
        if self.config.candidate_layout not in ('grid9', 'hybrid15'):
            raise ValueError(f'未知候选布局: {self.config.candidate_layout}')
        lattice = Lattice.build(self.config.candidate_spacing_m,
                                self.config.candidate_radius_m)
        points = lattice.points
        inside = np.hypot(points[:, 0], points[:, 1]) <= TARGET_RADIUS_M + 1e-9
        grid9 = points[inside]
        if self.config.candidate_layout == 'grid9':
            return grid9
        hex7 = hex_waypoints(self.config.candidate_radius_m)
        return np.unique(np.round(np.vstack((grid9, hex7)), decimals=9), axis=0)

    # ------------------------------------------------------------- 工具方法

    def _real_elapsed_s(self):
        """已经消耗的真实秒数（含本机计算时间）。"""
        return time.monotonic() - self._started_at

    def _action_capacity(self):
        """按实测每动作真实耗时估计还能发出的动作数。"""
        remaining = self.context.remaining_real_duration_s
        if remaining is None:
            return 1 << 20
        if not self.steps:
            return 1 << 20
        budget = max(0.0, remaining - self.config.real_time_reserve_s)
        per_action = max(self._real_elapsed_s() / len(self.steps), 1e-3)
        return int(budget / per_action)

    def _active_channels(self):
        return [channel for channel, knowledge in self.channels.items() if knowledge.is_active]

    def _detected_channels(self):
        return [channel for channel, knowledge in self.channels.items()
                if knowledge.status == 'detected' and knowledge.is_active]

    def _observed_channels(self):
        """已有任意读数的活跃频道：可靠/试探清除的候选来源。

        比 `_detected_channels` 更宽：只要该频道拿到过示向度、距离过近或无信号中
        任意一类读数，就进入清除判断，与 §3.5"根据最新信息逐频道判断"一致。
        无可读数的频道其位置集合仍是整个区域，必然不满足任何清除条件。
        """
        return [channel for channel, knowledge in self.channels.items()
                if knowledge.is_active and knowledge.observations]

    def _reach(self, waypoint):
        """候选停靠点的可检测格点集合（按停靠点缓存）。"""
        key = (round(float(waypoint[0]), 3), round(float(waypoint[1]), 3))
        if key not in self._reach_cache:
            inner = MIN_RECEIVE_RADIUS_M - self.fine.covering_radius_m
            self._reach_cache[key] = self.fine.distances_to(*key) <= inner
        return self._reach_cache[key]

    def _station_is_new(self, channel, waypoint):
        knowledge = self.channels[channel]
        return (knowledge.measured_at(waypoint[0], waypoint[1]) is None
                and not knowledge.cleared_here(waypoint[0], waypoint[1]))

    def _travel_m(self, waypoint):
        return float(np.hypot(waypoint[0] - self.position[0], waypoint[1] - self.position[1]))

    def _travel_s(self, waypoint):
        return self._travel_m(waypoint) / ROBOT_SPEED_MPS

    def _order_measures(self, channels):
        """先测当前频道再按编号升序，减少频道切换耗时。"""
        return sorted(set(channels), key=lambda c: (c != self.current_channel, c))

    def _measure_cost_s(self, channels):
        ordered = self._order_measures(channels)
        switch = 0
        previous = self.current_channel
        for channel in ordered:
            if channel != previous:
                switch += 1
            previous = channel
        return MEASURE_TIME_S * len(ordered) + CHANNEL_SWITCH_TIME_S * switch

    def _search_gain(self, channel, waypoint):
        """在 waypoint 检测该频道可新增排除的格点面积。"""
        count = int(np.count_nonzero(self.channels[channel].mask & self._reach(waypoint)))
        return count * self.fine.spacing_m ** 2

    def _search_options(self, waypoint, min_gain, cap):
        """一个停靠点上值得检测的频道：已发现频道全取，未发现频道按增益取前 cap 个。"""
        detected, searchable = [], []
        for channel in self._active_channels():
            if not self._station_is_new(channel, waypoint):
                continue
            gain = self._search_gain(channel, waypoint)
            if gain <= 0.0:
                continue
            if self.channels[channel].status == 'detected':
                detected.append((channel, gain))
            elif gain >= min_gain:
                searchable.append((channel, gain))
        searchable.sort(key=lambda item: -item[1])
        if cap is not None:
            searchable = searchable[:cap]
        return [channel for channel, _ in detected] + [channel for channel, _ in searchable]

    def _measure_gain(self, channels, waypoint):
        """一组检测在该停靠点可新增排除的总格点面积。"""
        return sum(self._search_gain(channel, waypoint) for channel in channels)

    def _search_cap(self):
        """真实时间容量不足时，减少单个停靠点的检测频道数。"""
        capacity = self._action_capacity()
        if capacity >= 60:
            return CHANNEL_COUNT
        if capacity >= 25:
            return 8
        return 3

    # -------------------------------------------------- 决策1+2：方向与步长

    def _chase_options(self):
        """已发现未清除频道的追捕选项，按最坏可能距离排序后只前瞻前若干个。"""
        detected = []
        for channel in self._detected_channels():
            limit = self.channels[channel].max_distance_m(*self.position)
            if limit is None:
                continue
            detected.append((limit, channel))
        if self.config.allocation_by_region:
            def region_cost(item):
                """按接近区域和剩余定位规模分配，严格判据仍用原距离上界。"""
                limit, channel = item
                center, radius = self.channels[channel].region_estimate(use_fine=True)
                return (limit if center is None else self._travel_m(center) + radius * 0.25, channel)
            detected.sort(key=region_cost)
        else:
            detected.sort()
        options = []
        for limit, channel in detected[:self.config.chase_options_limit]:
            knowledge = self.channels[channel]
            tracking = self.tracking_states.get(channel)
            if tracking is not None and tracking['upper_m'] <= CLEAR_RADIUS_M:
                waypoint = np.asarray(tracking['anchor'], dtype=float)
                options.append(StopPlan(
                    'clear', waypoint, [], self._travel_m(waypoint),
                    self._travel_s(waypoint) + CLEAR_TIME_S,
                    note=f'频道{channel}保证性追踪上界已进入清除半径',
                    target_channel=channel))
                continue
            if limit <= CLEAR_RADIUS_M:
                options.append(StopPlan('clear', np.array(self.position), [], 0.0, 0.0,
                                        note=f'频道{channel}已可证清除',
                                        target_channel=channel))
                continue
            if self.config.rendezvous_chase:
                options.extend(self._rendezvous_options(channel, knowledge))
            waypoint, note = self._chase_waypoint(channel, knowledge, limit)
            if waypoint is None or not self._station_is_new(channel, waypoint):
                continue
            measures = self._search_options(waypoint, self.config.search_min_gain_m2,
                                            self.config.search_channels_per_stop)
            # 到位后若已可证清除，就不再为该频道检测，直接进入清除动作。
            if not knowledge.certain_clear(waypoint[0], waypoint[1]) and channel not in measures:
                measures.append(channel)
            measures = self._order_measures(measures)
            if not measures and not knowledge.certain_clear(waypoint[0], waypoint[1]):
                continue
            score = self._clear_expectation_s(channel, waypoint, measures)
            options.append(StopPlan('chase', waypoint, measures, self._travel_m(waypoint),
                                    score_s=score, note=note, target_channel=channel))
        return options

    @staticmethod
    def _tracking_ratio():
        """返回示向误差上界下保证性追踪的距离收缩系数 q。"""
        return 1.0 / (2.0 * np.cos(np.deg2rad(ANGLE_ERROR_DEG)))

    def _rendezvous_options(self, channel, knowledge):
        """双站交会追击选项（2026-09-12 证伪，见 problem3_flow.md §10.2）：
        在首次示向锚点两侧对称、垂直于示向方向的 `rendezvous_offsets_m` 距离上
        布置第二检测站候选，与其余追击选项同一尺度评分。默认关闭。"""
        state = self.tracking_states.get(channel)
        if state is None or state['upper_m'] <= CLEAR_RADIUS_M:
            return []
        bearing_rad = np.deg2rad(float(state['bearing_deg']))
        perp = np.array([-np.sin(bearing_rad), np.cos(bearing_rad)])
        anchor = np.asarray(state['anchor'], dtype=float)
        options = []
        for sign in (-1.0, 1.0):
            for offset_m in self.config.rendezvous_offsets_m:
                waypoint = anchor + sign * offset_m * perp
                if np.hypot(*waypoint) > self.config.candidate_radius_m + 900.0:
                    continue
                if not self._station_is_new(channel, waypoint):
                    continue
                penalty = 0.0
                if self.config.receive_aware_chase:
                    penalty = self._receive_penalty(knowledge.plan_mask, self.coarse, waypoint)
                    if penalty is None:
                        continue
                measures = self._search_options(waypoint, self.config.search_min_gain_m2,
                                                self.config.search_channels_per_stop)
                if not knowledge.certain_clear(waypoint[0], waypoint[1]) and channel not in measures:
                    measures.append(channel)
                measures = self._order_measures(measures)
                if not measures:
                    continue
                score = (self._clear_expectation_s(channel, waypoint, measures) + penalty)
                options.append(StopPlan(
                    'chase', waypoint, measures, self._travel_m(waypoint), score_s=score,
                    note=f'频道{channel}双站交会{sign * offset_m:+.0f}m',
                    target_channel=channel))
        return options

    def _tracking_remaining_s(self, upper_m):
        """估计从安全距离上界 U 出发完成保证性追踪所需的保守虚拟时间。"""
        upper_m = max(0.0, float(upper_m))
        if upper_m <= CLEAR_RADIUS_M:
            return CLEAR_TIME_S
        ratio = self._tracking_ratio()
        steps = int(np.ceil(np.log(CLEAR_RADIUS_M / upper_m) / np.log(ratio)))
        travel_m = ratio * upper_m * (1.0 - ratio ** steps) / (1.0 - ratio)
        return travel_m / ROBOT_SPEED_MPS + MEASURE_TIME_S * steps + CLEAR_TIME_S

    def _guaranteed_tracking_plan(self, channel, knowledge):
        """由最新有效示向构造一次严格收缩距离上界的追踪动作。"""
        state = self.tracking_states.get(channel)
        if state is None or state['upper_m'] <= CLEAR_RADIUS_M:
            return None
        ratio = self._tracking_ratio()
        anchor = np.asarray(state['anchor'], dtype=float)
        bearing_rad = np.deg2rad(state['bearing_deg'])
        direction = np.array([np.cos(bearing_rad), np.sin(bearing_rad)])
        waypoint = anchor + ratio * state['upper_m'] * direction
        if not self._station_is_new(channel, waypoint):
            return None
        measures = self._search_options(
            waypoint, self.config.search_min_gain_m2,
            self.config.search_channels_per_stop)
        if channel not in measures:
            measures.append(channel)
        measures = self._order_measures(measures)
        next_upper = ratio * state['upper_m']
        score = (self._travel_s(waypoint) + self._measure_cost_s(measures)
                 + self._tracking_remaining_s(next_upper))
        return StopPlan(
            'guaranteed_track', waypoint, measures, self._travel_m(waypoint),
            score_s=score, note=f'频道{channel}保证性示向追踪',
            target_channel=channel, guaranteed_upper_m=next_upper)

    def _clear_expectation_s(self, channel, waypoint, measures):
        """追捕选项的期望耗时：移动 + 检测 + 期望剩余距离 + 末端定位清除。"""
        predicted = self._predicted_series(channel, waypoint)
        remaining = float(np.mean(predicted)) if predicted else 0.0
        return (self._travel_s(waypoint) + self._measure_cost_s(measures)
                + remaining / ROBOT_SPEED_MPS + self.config.endgame_seconds)

    def _chase_waypoint(self, channel, knowledge, limit):
        """决策2：给出追捕该频道时本次的停靠点。

        远场（最坏距离仍大）用粗格网前瞻；近场用细格网做局部前瞻，
        候选点包含"直接走到覆盖圆圆心"与若干带侧向偏置的停靠点，
        由"移动时间 + 预期最坏剩余距离 / 速度"自动权衡接近与交会角。
        """
        if limit <= self.config.local_lookahead_limit_m:
            return self._local_lookahead(knowledge, limit), '近场前瞻'
        return self._lookahead_waypoint(channel, knowledge), '远场前瞻'

    def _receive_penalty(self, mask, lattice, waypoint):
        """接收范围感知评分：返回惩罚秒数，None 表示该候选必然读不到、应跳过。

        检测点 $X$ 到可能位置的距离决定读数分支：距离 ≤ 1000 m（接收半径下界）
        时读数有保证；> 1500 m 时必然无信号且只排除空集，检测纯属浪费；
        (1000,1500] 是否可读取决于未知的 a，按保守口径视为读不到。无信号分支下
        机器狗仍需继续逼近，故以"读不到那部分位置的最坏距离 / 速度"按面积比例
        计入惩罚，不引入任何分布假设。
        """
        points = lattice.points[mask]
        if not len(points):
            return 0.0
        if len(points) > 4000:
            points = points[::int(np.ceil(len(points) / 4000))]
        distances = np.linalg.norm(points - np.asarray(waypoint, dtype=float), axis=1)
        if float(distances.min()) > MAX_RECEIVE_RADIUS_M + lattice.covering_radius_m:
            return None
        readable = distances <= MIN_RECEIVE_RADIUS_M + lattice.covering_radius_m
        if self.config.receive_aware_skip_unread and not readable.any():
            return None
        fraction = float(np.mean(readable))
        if fraction >= 1.0:
            return 0.0
        worst_unread = float(distances[~readable].max())
        return (self.config.receive_aware_penalty_scale
                * (1.0 - fraction) * worst_unread / ROBOT_SPEED_MPS)

    def _local_lookahead(self, knowledge, limit):
        """细格网局部前瞻：候选点少、假设点取覆盖圆圆心，单次决策保持毫秒级。"""
        center, radius = knowledge.region_estimate(use_fine=True)
        if center is None:
            return None
        step = min(limit, self.config.approach_direct_m)
        angles = np.arange(8) * 45.0
        local_directions = (adaptive_directions(self.position, center, 8)
                            if self.config.adaptive_initial_direction else
                            np.column_stack((np.cos(np.deg2rad(angles)), np.sin(np.deg2rad(angles)))))
        candidates = [np.asarray(center, dtype=float)]
        for scale in (0.5, 1.0):
            for direction in local_directions:
                candidates.append(np.array(self.position) + scale * step * direction)
        candidates.append(np.array(center, dtype=float) + np.array([radius, 0.0]))
        best, best_score = None, float('inf')
        for waypoint in bounded_candidates(candidates, self.position, self.config.tracking_path_limit_m):
            if not self._station_is_new(knowledge.channel, waypoint):
                continue
            penalty = 0.0
            if self.config.receive_aware_chase:
                penalty = self._receive_penalty(knowledge.mask, self.fine, waypoint)
                if penalty is None:
                    continue
            travel_s = float(np.hypot(waypoint[0] - self.position[0],
                                      waypoint[1] - self.position[1])) / ROBOT_SPEED_MPS
            total, count = 0.0, 0
            for hypothesis in (center,):
                for error in (-ANGLE_ERROR_DEG, ANGLE_ERROR_DEG):
                    bearing = float(bearing_deg(hypothesis - waypoint)) + error
                    predicted = apply_direction(knowledge.mask, self.fine,
                                                waypoint[0], waypoint[1], bearing)
                    if not predicted.any():
                        count = 0
                        break
                    remaining = np.linalg.norm(self.fine.points[predicted] - waypoint, axis=1)
                    total += float(np.max(remaining)) + self.fine.covering_radius_m
                    count += 1
                if count == 0:
                    break
            if count == 0:
                continue
            score = travel_s + (total / count) / ROBOT_SPEED_MPS + penalty
            if score < best_score:
                best, best_score = waypoint, score
        return best

    def _lookahead_waypoint(self, channel, knowledge):
        """粗格网单步前瞻：选择期望最坏剩余距离最小的停靠点。

        对每个候选停靠点，用可能位置与示向误差的抽样模拟下一次读数，在预测更新后的
        集合上取**最坏**剩余距离。取最坏值而非均值是必要的：沿示向线前移时预测集合
        仍是一条长条，其最坏距离不会下降，共线退化因此会被自动排除。
        """
        mask = knowledge.plan_mask
        if not mask.any():
            return None
        points = self.coarse.points[mask]
        step = max(1, len(points) // max(1, self.config.lookahead_hypotheses))
        hypotheses = points[::step][:self.config.lookahead_hypotheses]
        angles = np.arange(self.config.lookahead_directions) * (360.0 / self.config.lookahead_directions)
        directions = np.column_stack((np.cos(np.deg2rad(angles)), np.sin(np.deg2rad(angles))))
        if self.config.adaptive_initial_direction:
            center = (points.min(axis=0) + points.max(axis=0)) / 2
            directions = adaptive_directions(self.position, center, self.config.lookahead_directions)
        scored = []
        best, best_score = None, float('inf')
        for length in self.config.lookahead_lengths_m:
            if self.config.tracking_path_limit_m and length > self.config.tracking_path_limit_m:
                continue
            for direction in directions:
                waypoint = np.array(self.position) + length * direction
                if np.hypot(*waypoint) > self.config.candidate_radius_m + 900.0:
                    continue
                if not self._station_is_new(channel, waypoint):
                    continue
                penalty = 0.0
                if self.config.receive_aware_chase:
                    penalty = self._receive_penalty(mask, self.coarse, waypoint)
                    if penalty is None:
                        continue
                travel_s = length / ROBOT_SPEED_MPS
                total, count = 0.0, 0
                for source in hypotheses:
                    for error in (-ANGLE_ERROR_DEG, ANGLE_ERROR_DEG):
                        bearing = float(bearing_deg(source - waypoint)) + error
                        predicted = apply_direction(mask, self.coarse,
                                                    waypoint[0], waypoint[1], bearing)
                        if not predicted.any():
                            continue
                        remaining = np.linalg.norm(self.coarse.points[predicted] - waypoint, axis=1)
                        total += float(np.max(remaining)) + self.coarse.covering_radius_m
                        count += 1
                if count == 0:
                    continue
                score = travel_s + (total / count) / ROBOT_SPEED_MPS + penalty
                scored.append((score, waypoint))
                if score < best_score:
                    best, best_score = waypoint, score
        if (self.config.chase_rollout_topk > 0 and len(scored) > 1):
            return self._rollout_best(channel, mask, hypotheses, scored)
        return best

    def _rollout_best(self, channel, mask, hypotheses, scored):
        """二步滚动前瞻：对单步评分最好的前 K 个候选，模拟"检测后走第二步"。

        单步评分是短视的——它不知道检测完还能继续逼近。此处对前 K 个候选
        逐一模拟第二步（在预测掩码上再选一个最优停靠点），以
        "travel1 + min over 第二步(travel2 + 最坏剩余2)/速度" 重评并取最优。
        第二步候选集很小（chase_rollout_directions × lengths），只在 K 个候选
        上展开，单次决策保持毫秒级。
        """
        rollout_directions = np.arange(self.config.chase_rollout_directions) * (
            360.0 / self.config.chase_rollout_directions)
        dirs2 = np.column_stack((np.cos(np.deg2rad(rollout_directions)),
                                 np.sin(np.deg2rad(rollout_directions))))
        best, best_score = None, float('inf')
        for _, waypoint in sorted(scored, key=lambda item: item[0])[
                :self.config.chase_rollout_topk]:
            total, count = 0.0, 0
            for source in hypotheses:
                for error in (-ANGLE_ERROR_DEG, ANGLE_ERROR_DEG):
                    bearing = float(bearing_deg(source - waypoint)) + error
                    predicted = apply_direction(mask, self.coarse,
                                                waypoint[0], waypoint[1], bearing)
                    if not predicted.any():
                        continue
                    branch_best = None
                    for length2 in self.config.chase_rollout_lengths_m:
                        for direction in dirs2:
                            wp2 = waypoint + length2 * direction
                            if np.hypot(*wp2) > self.config.candidate_radius_m + 900.0:
                                continue
                            if not self._station_is_new(channel, wp2):
                                continue
                            bearing2 = float(bearing_deg(source - wp2)) + error
                            predicted2 = apply_direction(predicted, self.coarse,
                                                         wp2[0], wp2[1], bearing2)
                            if not predicted2.any():
                                continue
                            remaining2 = np.linalg.norm(
                                self.coarse.points[predicted2] - wp2, axis=1)
                            value = (length2 / ROBOT_SPEED_MPS
                                     + (float(np.max(remaining2))
                                        + self.coarse.covering_radius_m) / ROBOT_SPEED_MPS)
                            if branch_best is None or value < branch_best:
                                branch_best = value
                    if branch_best is None:
                        # 第二步无可行点时退回单步口径（只计剩余距离）。
                        remaining = np.linalg.norm(
                            self.coarse.points[predicted] - waypoint, axis=1)
                        branch_best = ((float(np.max(remaining))
                                        + self.coarse.covering_radius_m) / ROBOT_SPEED_MPS)
                    total += branch_best
                    count += 1
            if count == 0:
                continue
            score = (float(np.hypot(waypoint[0] - self.position[0],
                                    waypoint[1] - self.position[1])) / ROBOT_SPEED_MPS
                     + total / count)
            if score < best_score:
                best, best_score = waypoint, score
        return best

    def _predicted_series(self, channel, waypoint):
        """当前可能位置到 waypoint 的距离样本（粗格网），用于估计剩余距离。"""
        mask = self.channels[channel].plan_mask
        if not mask.any():
            return []
        return np.linalg.norm(self.coarse.points[mask] - waypoint, axis=1).tolist()

    # ------------------------------------------------------- 决策1：覆盖搜索

    def _search_plan(self):
        """覆盖搜索选项：以"每期望清除数所需秒数"评价候选停靠点。

        五段式：
        1. 按新增排除面积下限筛选（常规高增益搜索）；
        2. 去掉下限再找一次（只剩零星残差时的常规兜底）；
        3. 清除数达到题目下界且无待追踪源时，让动态空洞站点与固定格点竞争；
        4. 已清除至少 14 个源时，在无信号情景下前瞻完整补证路线，仅执行首站；
        5. **覆盖兜底**：若固定格点无方案而仍有整块残余空洞，直接以
           "补齐最大连通空洞"为目标选点。

        关于覆盖完备性（`verify_coverage_completeness` 可离线验证）：
        默认九点候选对任意源位置的最坏距离小于有效接收半径下界
        1000 m，因此**若某频道在全部候选点均无信号，则该频道确实不存在**——
        这是一条严格结论，不依赖轨迹偶然性。

        动态方案和第 5 段兜底仍然保留，理由是"候选点全测"要求该频道在每个点都被安排检测，
        而每站只取 `search_channels_per_stop` 个未发现频道；当活跃频道很多、
        时间预算又紧时，`_search_cap` 会进一步收紧每站检测数，理论上存在
        "某频道凑不齐 9 点就被判终止"的窗口。兜底段是这一窗口的安全网，
        同时也能在候选格网因配置改动而不再完备时自动补救。
        """
        # 承诺式补证游进行中：估计用的顺序与执行顺序一致（见 _tour_plan）。
        tour_plan = self._tour_plan()
        if tour_plan is not None:
            return tour_plan
        # 剩余源数的上界：既受总数上限约束，也不能超过"还没清除的频道数"。
        # 注意不能用 len(_active_channels())——它包含尚未判定排除的频道，
        # 会把剩余源数估高，进而高估搜索性价比、低估追击优先级。
        uncleared = CHANNEL_COUNT - len(self.cleared)
        remaining_sources = min(MAX_SOURCE_COUNT - len(self.cleared), uncleared)
        if remaining_sources <= 0:
            return None
        fixed_plan = None
        for min_gain in (self.config.search_min_gain_m2, 0.0):
            fixed_plan = self._best_search_plan(remaining_sources, min_gain)
            if fixed_plan is not None:
                break

        # 达到干扰源数量下界后，未知频道很可能已经不存在。此时优化目标从
        # “尽快发现新源”逐渐转为“用最短路线补齐排除证据”，允许动态空洞站点
        # 与固定覆盖格点竞争，避免固定九点在收尾阶段产生跨场折返。
        dynamic_plan = None
        if (self.config.enable_dynamic_endgame
                and len(self.cleared) >= self.config.dynamic_endgame_min_clears
                and not self._detected_channels()):
            dynamic_plan = self._backstop_search_plan(remaining_sources)

        # 已清除源较多时，“剩余频道实际不存在”已经是不可忽略的情景。此处在
        # 掩码副本上模拟无信号结果，估计完整补证路线并只执行最优路线的首站。
        # 若首站真实返回示向或近场，下一轮会立即退出本分支并恢复追踪。
        if (self.config.enable_route_aware_endgame
                and len(self.cleared) >= self.config.route_endgame_min_clears
                and not self._detected_channels()):
            route_plan = self._route_aware_endgame_plan(dynamic_plan)
            if route_plan is not None:
                return route_plan

        plans = [plan for plan in (fixed_plan, dynamic_plan) if plan is not None]
        if plans:
            return min(plans, key=lambda plan: plan.score_s)
        return self._backstop_search_plan(remaining_sources)

    def _route_aware_endgame_plan(self, dynamic_plan=None):
        """在无信号情景下估计完整补证路线，并返回预计总代价最小的首站。

        候选首站包含当前位置、固定覆盖格点以及当前动态空洞方案。对每个首站，
        在后验掩码副本上假设相关频道返回无信号，再用最近必要站规则补齐固定
        覆盖点。模拟过程绝不修改真实知识状态或覆盖证书。
        """
        active = [channel for channel in self._active_channels()
                  if self.channels[channel].status == 'unknown']
        if not active:
            return None
        first_candidates = list(self._candidate_waypoints())
        if dynamic_plan is not None:
            first_candidates.append(np.asarray(dynamic_plan.waypoint, dtype=float))
        first_candidates = np.unique(
            np.round(np.asarray(first_candidates, dtype=float), decimals=6), axis=0)

        best = None
        best_total_s = float('inf')
        for waypoint in first_candidates:
            measures = self._search_options(waypoint, 0.0, None)
            measures = [channel for channel in measures if channel in active]
            measures = measures[:max(1, self._search_cap())]
            if not measures:
                continue
            if self.config.route_planner == 'dijkstra':
                total_s = self._dijkstra_absence_route_seconds(waypoint, measures, active)
            else:
                total_s = self._absence_route_seconds(waypoint, measures, active)
            if total_s < best_total_s:
                best_total_s = total_s
                best = StopPlan(
                    'route_backstop', np.asarray(waypoint, dtype=float), measures,
                    self._travel_m(waypoint), score_s=total_s,
                    note='路线前瞻兜底：执行无信号情景最短补证路线的首站')
        # 承诺式补证游：对最终选中的首站重模拟一次，缓存 2-opt 后的完整
        # 路线。此后每轮直接按序执行下一站，直到出现新发现或路线耗尽——
        # 保证"估计用的顺序"与"实际执行的顺序"一致。
        if (best is not None and self.config.absence_tour_commit
                and self.config.route_planner != 'dijkstra'):
            _, sequence = self._absence_route_seconds(
                best.waypoint, best.measures, active, return_sequence=True)
            self.absence_tour = sequence
            self.absence_tour_pos = 0
        return best

    def _tour_plan(self):
        """返回承诺式补证游的下一站计划；路线失效或耗尽时返回 None。"""
        if (not self.config.absence_tour_commit
                or not self.absence_tour
                or self.absence_tour_pos >= len(self.absence_tour)):
            return None
        while self.absence_tour_pos < len(self.absence_tour):
            waypoint, channels = self.absence_tour[self.absence_tour_pos]
            pending = [channel for channel in channels
                       if self.channels[channel].is_active
                       and self._station_is_new(channel, waypoint)]
            if pending:
                remaining_s = self._travel_s(waypoint) + self._measure_cost_s(pending)
                return StopPlan(
                    'tour_station', np.asarray(waypoint, dtype=float), pending,
                    self._travel_m(waypoint), score_s=remaining_s,
                    note=f'承诺补证游第{self.absence_tour_pos + 1}/'
                         f'{len(self.absence_tour)}站')
            self.absence_tour_pos += 1
        return None

    def _absence_route_seconds(self, first_waypoint, first_measures, active_channels,
                               return_sequence=False):
        """在掩码副本上模拟无信号补证路线，返回移动与测量总时间估计。

        后续每一步仅在固定覆盖格点中选择距离当前位置最近、且仍能排除残余
        区域的站点。固定格点具有完整覆盖证明，因此模拟若正常结束必能清空掩码。
        `absence_route_2opt` 开启时，贪心序列构造完成后对访问顺序做 2-opt
        改进（站点集合与各站检测频道不变，覆盖结果不变），返回两种顺序中
        较小的总耗时。
        """
        masks = {channel: self.channels[channel].mask.copy()
                 for channel in active_channels}
        position = np.asarray(self.position, dtype=float)
        total_s = 0.0
        visited = set()
        simulated_channel = self.current_channel
        visit_sequence = []

        def visit(waypoint, channels):
            """在模拟副本上访问一个站点，并累计移动与测量时间。"""
            nonlocal position, simulated_channel, total_s
            waypoint = np.asarray(waypoint, dtype=float)
            total_s += float(np.linalg.norm(waypoint - position)) / ROBOT_SPEED_MPS
            ordered = sorted(set(channels),
                             key=lambda channel: (channel != simulated_channel, channel))
            switches = 0
            previous = simulated_channel
            for channel in ordered:
                if channel != previous:
                    switches += 1
                previous = channel
            total_s += MEASURE_TIME_S * len(ordered) + CHANNEL_SWITCH_TIME_S * switches
            if ordered:
                simulated_channel = ordered[-1]
            reach = self._reach(waypoint)
            for channel in ordered:
                masks[channel] &= ~reach
            position = waypoint
            visited.add((round(float(waypoint[0]), 6), round(float(waypoint[1]), 6)))
            visit_sequence.append((waypoint, ordered))

        visit(first_waypoint, first_measures)
        for _ in range(len(self.candidates)):
            if not any(mask.any() for mask in masks.values()):
                sequence = None
                if self.config.absence_route_2opt and len(visit_sequence) > 2:
                    improved = self._two_opt_route_seconds(visit_sequence)
                    if improved is not None and improved < total_s:
                        sequence = self._two_opt_order(visit_sequence)
                        total_s = improved
                if return_sequence:
                    return total_s, (sequence or list(visit_sequence))
                return total_s
            options = []
            for waypoint in self.candidates:
                key = (round(float(waypoint[0]), 6), round(float(waypoint[1]), 6))
                if key in visited:
                    continue
                reach = self._reach(waypoint)
                channels = [channel for channel, mask in masks.items()
                            if np.any(mask & reach)]
                if not channels:
                    continue
                distance_m = float(np.linalg.norm(np.asarray(waypoint) - position))
                gain = sum(int(np.count_nonzero(masks[channel] & reach))
                           for channel in channels)
                options.append((distance_m, -gain, waypoint, channels))
            if not options:
                break
            _, _, waypoint, channels = min(options, key=lambda item: (item[0], item[1]))
            visit(waypoint, channels)
        return float('inf')

    def _two_opt_order(self, visit_sequence):
        """返回 2-opt 改进后的访问顺序（不重算时间，只重排索引）。"""
        if self.config.rolling_search_route:
            order = open_route_order([point for point, _ in visit_sequence],
                                     self.position, list(range(len(visit_sequence))))
            return [visit_sequence[index] for index in order]
        start = np.asarray(self.position, dtype=float)
        points = [np.asarray(waypoint, dtype=float) for waypoint, _ in visit_sequence]
        count = len(points)
        order = list(range(count))

        def path_length(sequence):
            previous = start
            total = 0.0
            for index in sequence:
                total += float(np.linalg.norm(points[index] - previous))
                previous = points[index]
            return total

        improved = True
        while improved:
            improved = False
            for i in range(count - 1):
                for j in range(i + 1, count):
                    candidate = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                    if path_length(candidate) < path_length(order) - 1e-6:
                        order = candidate
                        improved = True
        return [visit_sequence[index] for index in order]

    def _two_opt_route_seconds(self, visit_sequence):
        """对无信号补证路线的访问顺序做 2-opt，返回改进后的总耗时估计。

        只重排访问顺序：各站点的检测频道集合不变，因此掩码清空结果与原
        序列完全一致；总耗时按新顺序重算（行进 + 测量 + 频道切换）。
        """
        start = np.asarray(self.position, dtype=float)
        reordered = self._two_opt_order(visit_sequence)
        total_s = 0.0
        position = start
        simulated_channel = self.current_channel
        for waypoint, ordered_channels in reordered:
            total_s += float(np.linalg.norm(waypoint - position)) / ROBOT_SPEED_MPS
            previous = simulated_channel
            switches = 0
            for channel in ordered_channels:
                if channel != previous:
                    switches += 1
                previous = channel
            total_s += (MEASURE_TIME_S * len(ordered_channels)
                        + CHANNEL_SWITCH_TIME_S * switches)
            if ordered_channels:
                simulated_channel = ordered_channels[-1]
            position = waypoint
        return total_s

    def _dijkstra_absence_route_seconds(self, first_waypoint, first_measures, active_channels):
        """用插入启发式 + Dijkstra 距离优化无信号补证路线。

        与贪婪最近站不同，此处把「固定覆盖格点 + 首站」看成一个待访问序列，
        每次把能覆盖最多残余掩码、且边际代价最低的点插入当前序列的最优位置。
        序列内部相邻点之间的距离用 Dijkstra（本场景即欧氏直线）计算。
        """
        masks = {channel: self.channels[channel].mask.copy()
                 for channel in active_channels}
        simulated_channel = self.current_channel

        def route_cost_and_gain(route):
            """计算给定访问序列的总时间与最终覆盖收益（仅作比较，不求精确）。"""
            pos = np.asarray(self.position, dtype=float)
            chan = simulated_channel
            total = 0.0
            covered_cells = 0
            for waypoint in route:
                waypoint = np.asarray(waypoint, dtype=float)
                total += float(np.linalg.norm(waypoint - pos)) / ROBOT_SPEED_MPS
                reach = self._reach(waypoint)
                channels = [c for c, m in masks.items() if np.any(m & reach)]
                if not channels:
                    channels = active_channels
                ordered = sorted(set(channels), key=lambda c: (c != chan, c))
                switches = sum(1 for i, c in enumerate(ordered) if i == 0 or c != ordered[i - 1])
                total += MEASURE_TIME_S * len(ordered) + CHANNEL_SWITCH_TIME_S * switches
                chan = ordered[-1] if ordered else chan
                covered_cells += sum(int(np.count_nonzero(masks[c] & reach)) for c in active_channels)
                pos = waypoint
            return total, covered_cells

        def insert_cost(route, idx, waypoint):
            """把 waypoint 插入 route 的 idx 位置所带来的新增时间。"""
            prev = route[idx - 1] if idx > 0 else np.asarray(self.position, dtype=float)
            nxt = route[idx] if idx < len(route) else None
            added = float(np.linalg.norm(waypoint - prev)) / ROBOT_SPEED_MPS
            if nxt is not None:
                added += float(np.linalg.norm(nxt - waypoint)) / ROBOT_SPEED_MPS
                added -= float(np.linalg.norm(nxt - prev)) / ROBOT_SPEED_MPS
            return added

        def cells_gained(waypoint):
            reach = self._reach(waypoint)
            return sum(int(np.count_nonzero(masks[c] & reach)) for c in active_channels)

        route = [np.asarray(first_waypoint, dtype=float)]
        # 候选池：固定覆盖格点
        pool = [np.asarray(w, dtype=float) for w in self.candidates]
        for _ in range(len(self.candidates) + 1):
            if not any(mask.any() for mask in masks.values()):
                break
            best, best_score = None, float('inf')
            for waypoint in pool:
                gain = cells_gained(waypoint)
                if gain <= 0:
                    continue
                for idx in range(len(route) + 1):
                    cost = insert_cost(route, idx, waypoint)
                    # 目标：单位收益边际时间最小；加小量避免除以零。
                    score = cost / (gain + 1.0)
                    if score < best_score:
                        best_score = score
                        best = (idx, waypoint, gain)
            if best is None:
                break
            idx, waypoint, gain = best
            route.insert(idx, waypoint)
            reach = self._reach(waypoint)
            for c in active_channels:
                masks[c] &= ~reach

        total_s, _ = route_cost_and_gain(route)
        return total_s

    def verify_coverage_completeness(self, margin_m=0.0):
        """离线自检：候选停靠点是否足以对任意源位置给出"无信号"或"发现"。

        返回 (完备?, 最坏距离m, 依据)。
        判据：目标圆域内任意点到最近候选点的距离 <= MIN_RECEIVE_RADIUS_M 时，
        该点的源在任何一次检测中都不可能被漏过（有效接收半径 >= 1000 m）。
        因此"全部候选点均无信号"即为严格的不存在性证明。
        """
        from .config import MIN_RECEIVE_RADIUS_M
        # 在目标圆域上密采样，求到最近候选点的最大距离。
        angles = np.linspace(0.0, 2.0 * np.pi, 1441)
        radii = np.linspace(0.0, TARGET_RADIUS_M, 721)
        grid_angles, grid_radii = np.meshgrid(angles, radii)
        samples = np.column_stack((grid_radii.ravel() * np.cos(grid_angles.ravel()),
                                   grid_radii.ravel() * np.sin(grid_angles.ravel())))
        candidates = self.candidates
        in_range = candidates[np.hypot(candidates[:, 0], candidates[:, 1]) <= TARGET_RADIUS_M + 1e-9]
        worst = float(np.max(np.min(np.linalg.norm(samples[:, None, :] - in_range[None, :, :],
                                                   axis=2), axis=1)))
        complete = worst + margin_m <= MIN_RECEIVE_RADIUS_M
        return complete, worst, f'域内候选 {len(in_range)} 点，最坏距离 {worst:.2f} m'

    def _backstop_search_plan(self, remaining_sources):
        """覆盖兜底：朝着**最大连通残余空洞**走，而不是朝着任意残余格点走。

        与 `_best_search_plan` 的区别是候选点不再限于固定格网，而是在残余
        格点上直接选取，因此不会因格网太粗而漏掉空洞。

        关键设计：残余格点往往是"大片空洞 + 零星毛刺"的混合。若直接在全部
        残余格点上取候选，会为了几十个孤立格点横穿全场（实测行进反升 19%）。
        因此先把残余格点**按连通性聚类**，只取最大的一簇（真正需要跑一趟的
        那块空洞），再在其上选点。
        """
        active = self._active_channels()
        if not active:
            return None
        # 汇总所有活跃频道的残余掩码，得到"仍有待排除问题"的格点并集。
        pending = np.zeros(len(self.fine.points), dtype=bool)
        for channel in active:
            pending |= self.channels[channel].mask
        if not np.any(pending):
            return None
        stride_m = self.fine.spacing_m
        clusters = self._connected_clusters(self.fine.points[pending], stride_m * 1.5)
        if not clusters:
            return None
        # 只处理最大的一簇；小于阈值说明已无整块空洞，不必专门跑。
        clusters.sort(key=len, reverse=True)
        hole = clusters[0]
        hole_area = len(hole) * stride_m ** 2
        if hole_area < self.config.backstop_min_hole_m2:
            return None
        # 候选不再依赖固定九点：在当前最大残余空洞上做最远点采样，形成随
        # 后验掩码变化的动态站点。相比按数组下标等距抽样，最远点采样能稳定
        # 覆盖狭长、月牙形和偏心空洞，不会漏掉几何上关键的端部。
        sample = self._farthest_point_candidates(
            hole, self.config.backstop_max_stations)
        nearest = hole[np.argsort(np.linalg.norm(hole - np.array(self.position), axis=1))[:4]]
        # 再沿当前位置到空洞重心的方向补几个中间点，避免"一步到位"式长跳：
        # 中间点让策略有机会在通往空洞的路上顺带取得读数。
        centroid = hole.mean(axis=0)
        position = np.asarray(self.position, dtype=float)
        bridges = []
        for ratio in (0.35, 0.7):
            bridges.append(position + ratio * (centroid - position))
        candidates = np.vstack((sample, nearest, centroid[None, :], np.asarray(bridges)))
        # 数值格点与插值桥点可能重复；统一去重并确保站点位于题目目标圆域内。
        candidates = np.unique(np.round(candidates, decimals=6), axis=0)
        inside = np.hypot(candidates[:, 0], candidates[:, 1]) <= TARGET_RADIUS_M + 1e-9
        candidates = candidates[inside]
        best, best_score = None, float('inf')
        for waypoint in candidates:
            measures = self._search_options(waypoint, 0.0, None)
            if not measures:
                continue
            measures = measures[:max(1, self._search_cap())]
            gain = self._measure_gain(measures, waypoint)
            if gain <= 0.0:
                continue
            expected = self._expected_finds(remaining_sources, gain)
            if expected <= 0.0:
                continue
            follow_up = self._follow_up_seconds(waypoint) + self.config.endgame_seconds
            # 兜底选项不乘风险溢价：它的目的是"把结论做完整"，本身就是收益。
            score = (self._travel_s(waypoint) + self._measure_cost_s(measures)) / expected + follow_up
            if score < best_score:
                best = StopPlan('backstop', waypoint, measures, self._travel_m(waypoint),
                                score_s=score, expect_finds=expected,
                                note='覆盖兜底：补齐连通残余空洞')
                best_score = score
        return best

    @staticmethod
    def _farthest_point_candidates(points, limit):
        """对残余空洞做确定性最远点采样，返回几何分散的动态候选站点。

        首点取最接近空洞质心的格点；之后每次加入距已选集合最远的格点，
        相当于小规模 k-center 贪心。为控制实时规划耗时，超大空洞先均匀下采样。
        """
        points = np.asarray(points, dtype=float)
        limit = max(1, int(limit))
        if len(points) <= limit:
            return points.copy()
        if len(points) > 12000:
            stride = int(np.ceil(len(points) / 12000))
            pool = points[::stride]
        else:
            pool = points
        centroid = np.mean(pool, axis=0)
        first = int(np.argmin(np.linalg.norm(pool - centroid, axis=1)))
        selected = [first]
        min_distance_sq = np.sum((pool - pool[first]) ** 2, axis=1)
        for _ in range(1, min(limit, len(pool))):
            next_index = int(np.argmax(min_distance_sq))
            selected.append(next_index)
            distance_sq = np.sum((pool - pool[next_index]) ** 2, axis=1)
            min_distance_sq = np.minimum(min_distance_sq, distance_sq)
        return pool[np.asarray(selected, dtype=int)]

    @staticmethod
    def _connected_clusters(points, link_m):
        """把格点按欧氏距离阈值做连通聚类，返回每簇的点坐标列表。

        用并查集按 `link_m` 连接；格点规模十万级但残余格点通常很少，
        因此先做一次分块避免 O(n^2) 全比较。
        """
        count = len(points)
        if count == 0:
            return []
        if count > 20000:
            # 残余过多时不做精细聚类，直接整体视为一簇。
            return [points]
        parent = np.arange(count)

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        # 以 link_m 为格边长分桶，只需与本桶及邻接桶比较。
        keys = np.floor(points / link_m).astype(np.int64)
        buckets = {}
        for index, key in enumerate(map(tuple, keys)):
            buckets.setdefault(key, []).append(index)
        neighbors = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
        for key, members in buckets.items():
            for dx, dy in neighbors:
                other = buckets.get((key[0] + dx, key[1] + dy))
                if not other:
                    continue
                for i in members:
                    for j in other:
                        if i < j and np.linalg.norm(points[i] - points[j]) <= link_m:
                            root_i, root_j = find(i), find(j)
                            if root_i != root_j:
                                parent[root_j] = root_i
        groups = {}
        for index in range(count):
            groups.setdefault(find(index), []).append(index)
        return [points[np.asarray(index_list)] for index_list in groups.values()]

    def _best_search_plan(self, remaining_sources, min_gain):
        active = self._active_channels()
        best, best_score = None, float('inf')
        for waypoint in self._candidate_waypoints():
            if not any(self._station_is_new(channel, waypoint) for channel in active):
                continue
            qualifying = self._search_options(waypoint, min_gain, None)
            if not qualifying:
                continue
            # 选点仍由 `min_gain` 控制，防止为零星毛刺专程移动；但移动一旦确定，
            # 额外测一个频道只需 5--6 秒，通常远小于日后数百秒的跨场折返。
            measures = (self._search_options(waypoint, 0.0, None)
                        if self.config.fill_search_stops else qualifying)
            measures = measures[:max(1, self._search_cap())]
            gain = self._measure_gain(measures, waypoint)
            expected = self._expected_finds(remaining_sources, gain)
            if expected <= 0.0:
                continue
            # 新发现干扰源的后续清除代价也要计入，否则会高估搜索的性价比。
            follow_up = self._follow_up_seconds(waypoint) + self.config.endgame_seconds
            # 回访惩罚：该停靠点附近若已服务过（已在该处取得过读数），
            # 说明这片区域的排除收益大概率已被吃掉，压低其优先级，
            # 避免收尾阶段在同一批点之间反复横跳。
            revisit = self._revisit_penalty(waypoint)
            score = ((self._travel_s(waypoint) + self._measure_cost_s(measures)) / expected
                     + follow_up + revisit)
            score *= self.config.search_risk_premium
            if score < best_score:
                best = StopPlan('sweep', waypoint, measures, self._travel_m(waypoint),
                                score_s=score, expect_finds=expected)
                best_score = score
        return best

    def _revisit_penalty(self, waypoint):
        """停靠点回访惩罚（秒）：附近已服务过的停靠点越多，惩罚越大。

        "已服务"定义为该点周围 `revisit_radius_m` 内，存在任何频道留下过读数。
        这是纯粹的排序修正，不影响任何严格结论。
        """
        radius = self.config.revisit_radius_m
        if radius <= 0.0:
            return 0.0
        hits = 0
        for knowledge in self.channels.values():
            for px, py in knowledge.measure_positions:
                if np.hypot(px - waypoint[0], py - waypoint[1]) <= radius:
                    hits += 1
                    break
        return self.config.revisit_penalty_s * hits

    def _candidate_waypoints(self):
        """搜索候选停靠点：当前点与远层固定格网。

        这里**刻意不加近场候选**。做过的消融实验表明：近场候选（300~600 m）
        在 `_best_search_plan` 的评分下总是占优——因为分值分母是"期望发现数"、
        分子只有很短的行进时间，于是策略退化成"一步一挪"的短跳，
        12 源种子 7 实测行进从 19351 m 涨到 23029 m（+19%），虚拟时间 +14%。
        近场补刀的正确入口是 `_backstop_search_plan`（只在确有连通空洞时触发），
        而不是让常规搜索也能选近场点。
        """
        return np.vstack((np.asarray(self.position, dtype=float), self.candidates))

    def _expected_finds(self, remaining_sources, gain_m2):
        """均匀先验下的期望发现数：保持既有评分尺度，避免搜索/追击失衡。"""
        if gain_m2 <= 0.0:
            return 0.0
        return remaining_sources / CHANNEL_COUNT * gain_m2 / TARGET_AREA_M2

    def _follow_up_seconds(self, waypoint):
        """在 waypoint 新发现一个目标后，还需要多少移动时间。

        只对该停靠点**能覆盖到的区域**取平均（`_reach`），而不是整个目标圆域：
        后者对任何停靠点都给出同一个常数，等于没参与评分。真正决定后续代价的
        是"该站点接收圆内若发现源，平均还要追多远"。
        """
        reachable = self._reach(waypoint)
        if not np.any(reachable):
            return 0.0
        points = self.fine.points[reachable]
        # 大范围时抽样，避免每次评分都做十万级范数运算。
        if len(points) > 4000:
            points = points[::int(np.ceil(len(points) / 4000))]
        remaining = np.linalg.norm(points - np.asarray(waypoint, dtype=float), axis=1)
        return float(np.mean(remaining)) / ROBOT_SPEED_MPS

    # --------------------------------------------------------- 决策1 汇总

    def _decide(self):
        """决策1+2：在所有选项中选择"期望每秒清除数"最大者。"""
        try:
            options = self._chase_options()
            search = None
            if not (self.config.finish_detected_before_search and options):
                search = self._search_plan()
        except Exception:
            # 规划层异常不终止任务：记录后交回主循环，由停滞计数安全收尾。
            self.planner_errors += 1
            return None
        if search is not None:
            options.append(search)
        # `_chase_options` 为控制实时计算只前瞻少量频道；到期的保证性追踪必须
        # 越过这个截断，否则远距离频道可能一直排不进候选集。
        represented = {plan.target_channel for plan in options
                       if plan.kind == 'guaranteed_track'}
        for channel, wait_rounds in self.tracking_wait_rounds.items():
            if (not self.config.enable_guaranteed_tracking
                    or wait_rounds < self.config.tracking_debt_limit_rounds
                    or channel in represented
                    or not self.channels[channel].is_active):
                continue
            guaranteed = self._guaranteed_tracking_plan(channel, self.channels[channel])
            if guaranteed is not None:
                options.append(guaranteed)
        options = [plan for plan in options if np.isfinite(plan.score_s)]
        if not options:
            return None
        options = self._apply_turn_penalty(options)
        # 自适应评分不得无限推迟保证性任务：等待达到上限的追踪频道强制服务。
        due_tracking = [
            plan for plan in options
            if (plan.kind == 'guaranteed_track'
                and self.tracking_wait_rounds.get(plan.target_channel, 0)
                >= self.config.tracking_debt_limit_rounds)
        ]
        if due_tracking:
            due_tracking.sort(key=lambda plan: (
                -self.tracking_wait_rounds.get(plan.target_channel, 0),
                plan.score_s,
                plan.target_channel))
            return due_tracking[0]
        options.sort(key=lambda plan: plan.score_s)
        best = options[0]
        # 若最优计划需要长距离移动，检查直线路径上是否有高价值中间停靠点。
        intermediate = self._maybe_intermediate_plan(best)
        return intermediate if intermediate is not None else best

    def _apply_turn_penalty(self, options):
        """折返抑制：给与上一段移动方向相反的长距离选项加评分惩罚。

        判据只用"已实际发生的移动方位"与"候选点方位"的夹角，不引入任何预测或
        分布假设；因此它只改变选项之间的排序，不影响覆盖证书、清除判据或结束条件。
        `turn_penalty_s <= 0`（默认）时原样返回，行为与历史版本完全一致。
        """
        penalty_s = float(self.config.turn_penalty_s)
        if penalty_s <= 0.0 or self.last_move_bearing_deg is None:
            return options
        threshold = float(self.config.turn_penalty_angle_deg)
        min_leg = float(self.config.turn_penalty_min_leg_m)
        adjusted = []
        for plan in options:
            leg = self._travel_m(plan.waypoint)
            if leg < min_leg:
                adjusted.append(plan)
                continue
            bearing = float(np.degrees(np.arctan2(
                plan.waypoint[1] - self.position[1],
                plan.waypoint[0] - self.position[0])))
            delta = abs((bearing - self.last_move_bearing_deg + 180.0) % 360.0 - 180.0)
            if delta >= threshold:
                adjusted.append(replace(plan, score_s=plan.score_s + penalty_s,
                                        note=f'{plan.note} [折返{delta:.0f}°]'.strip()))
            else:
                adjusted.append(plan)
        return adjusted

    def _maybe_intermediate_plan(self, plan):
        """当计划移动距离较长时，在直线路径上采样中间点并择优停靠。

        价值判据：中间点能覆盖的未知粗格网面积 >= 阈值，且带来的期望发现数
        足以抵消额外的检测时间。若无可行中间点，返回 None，保持原 plan。
        """
        gap = self.config.intermediate_stop_gap_m
        if gap <= 0.0:
            return None
        start = np.asarray(self.position, dtype=float)
        target = np.asarray(plan.waypoint, dtype=float)
        vector = target - start
        length = float(np.linalg.norm(vector))
        if length <= gap:
            return None
        # 沿直线按 gap 间隔采样，避开起点和终点。
        n = int(np.floor(length / gap))
        best, best_score = None, float('inf')
        unit = vector / length
        for k in range(1, n + 1):
            waypoint = start + unit * (k * gap)
            if np.hypot(*waypoint) > TARGET_RADIUS_M + 1e-9:
                continue
            active = [c for c, k in self.channels.items()
                      if k.is_active and self._station_is_new(c, waypoint)]
            if not active:
                continue
            measures = self._search_options(waypoint, 0.0, None)
            measures = [c for c in measures if c in active]
            if not measures:
                continue
            gain = self._measure_gain(measures, waypoint)
            if gain < self.config.intermediate_stop_min_gain_m2:
                continue
            # 评估这个中间点：额外代价是偏离直线的折返 + 检测时间。
            detour = (float(np.linalg.norm(waypoint - start))
                      + float(np.linalg.norm(target - waypoint))
                      - length) / ROBOT_SPEED_MPS
            cost = detour + self._measure_cost_s(measures)
            expected = self._expected_finds(
                min(MAX_SOURCE_COUNT - len(self.cleared), len(active)), gain)
            if expected <= 0.0:
                continue
            score = cost / expected
            if score < best_score:
                best_score = score
                best = StopPlan('intermediate', waypoint, measures,
                                self._travel_m(waypoint), score_s=score,
                                note=f'长途移动中间检测：偏离{detour:.1f}s，期望发现{expected:.2f}')
        # 只替换原 plan 当且仅当中间点的性价比优于原 plan。
        if best is not None and best.score_s < plan.score_s:
            return best
        return None

    # --------------------------------------------------------- 决策3/4：执行

    def _check_action_budget(self, waypoint, kind, channel):
        """与问题四一致，发令前计入移动及动作总耗时，禁止跨越虚拟时限。"""
        state = self.context.state
        limit = getattr(state, 'max_virtual_duration_s', settings.VIRTUAL_BUDGET_S)
        action_s = (MEASURE_TIME_S + CHANNEL_SWITCH_TIME_S * (channel != self.current_channel)
                    if kind == 'measure' else settings.OPTICAL_TIME_S + CLEAR_TIME_S)
        if state.virtual_time_s + self._travel_s(waypoint) + action_s >= limit:
            raise BudgetReached('问题三本次动作将耗尽虚拟时间预算。')

    def _execute_measures(self, plan):
        acted = False
        x, y = float(plan.waypoint[0]), float(plan.waypoint[1])
        for channel in self._order_measures(plan.measures):
            if self.context.should_stop():
                break
            if not self._station_is_new(channel, plan.waypoint):
                continue
            self._check_action_budget(plan.waypoint, 'measure', channel)
            response = self.context.measure(x, y, channel)
            guaranteed_upper = (
                plan.guaranteed_upper_m
                if plan.kind == 'guaranteed_track' and channel == plan.target_channel
                else None)
            self._apply_measure(
                channel, plan.waypoint, response,
                guaranteed_upper_m=guaranteed_upper)
            acted = True
        if (plan.kind in ('tour_station', 'route_backstop')
                and self.absence_tour is not None
                and self.absence_tour_pos < len(self.absence_tour)
                and np.allclose(np.asarray(plan.waypoint, dtype=float),
                                np.asarray(self.absence_tour[self.absence_tour_pos][0],
                                           dtype=float), atol=1e-6)):
            self.absence_tour_pos += 1
        return acted

    def _apply_measure(self, channel, waypoint, response, guaranteed_upper_m=None):
        """应用检测反馈，并同步更新保证性追踪的锚点、示向和距离上界。"""
        self._move_to(waypoint)
        knowledge = self.channels[channel]
        result = response['measure_result']
        virtual = float(response['virtual_time_s'])
        if result == 'direction':
            bearing = float(response['svd_deg'])
            knowledge.observe_direction(waypoint[0], waypoint[1], bearing, virtual)
            # 普通有效示向由物理接收上界给出 U<=1500；若本次来自保证性追踪，
            # 余弦定理给出的 qU 更紧，且与随机场景和位置先验无关。
            upper = (MAX_RECEIVE_RADIUS_M
                     if guaranteed_upper_m is None else float(guaranteed_upper_m))
            self.tracking_states[channel] = {
                'anchor': (float(waypoint[0]), float(waypoint[1])),
                'bearing_deg': bearing,
                'upper_m': upper,
            }
            self.tracking_wait_rounds[channel] = 0
        elif result == 'near':
            knowledge.observe_near(waypoint[0], waypoint[1], virtual)
            self.tracking_states.pop(channel, None)
            self.tracking_wait_rounds[channel] = 0
        else:
            knowledge.observe_no_signal(waypoint[0], waypoint[1], virtual)
        if result in ('direction', 'near'):
            # 出现新发现即推翻"剩余频道均无信号"的补证游前提。
            self.absence_tour = None
        self.current_channel = channel
        self._record(channel, 'measure', result, response)
        self._sample_channel(channel)

    def _execute_clears(self, waypoint):
        acted = False
        x, y = float(waypoint[0]), float(waypoint[1])
        for channel in self._clear_candidates(waypoint):
            if self.context.should_stop():
                break
            self._check_action_budget(waypoint, 'clear', channel)
            response = self.context.clear(x, y, channel)
            self._apply_clear(channel, waypoint, response)
            acted = True
        return acted

    def _clear_candidates(self, waypoint):
        """决策4：以第三项结束后的最新信息判断是否清除。"""
        certain, speculative = [], []
        for channel in self._observed_channels():
            knowledge = self.channels[channel]
            if knowledge.cleared_here(waypoint[0], waypoint[1]):
                continue
            if (knowledge.certain_clear(waypoint[0], waypoint[1])
                    or knowledge.near_certain_clear(waypoint[0], waypoint[1])
                    or self._tracking_certain_clear(channel, waypoint)):
                certain.append(channel)
                continue
            limit = knowledge.max_distance_m(waypoint[0], waypoint[1])
            if limit is None or limit > self.config.speculative_clear_m:
                continue
            probability = knowledge.clear_probability(waypoint[0], waypoint[1])
            if probability >= self.config.speculative_min_probability:
                speculative.append((probability, channel))
        speculative.sort(key=lambda item: -item[0])
        return certain + [channel for _, channel in speculative]

    def _tracking_certain_clear(self, channel, waypoint):
        """判断保证性追踪上界是否已在当前停靠点进入 20 m 清除半径。"""
        state = self.tracking_states.get(channel)
        if state is None or state['upper_m'] > CLEAR_RADIUS_M:
            return False
        anchor = np.asarray(state['anchor'], dtype=float)
        return bool(np.linalg.norm(anchor - np.asarray(waypoint, dtype=float)) <= 1e-6)

    def _apply_clear(self, channel, waypoint, response):
        self._move_to(waypoint)
        knowledge = self.channels[channel]
        virtual = float(response['virtual_time_s'])
        if response['clear_result'] == 'success':
            knowledge.mark_cleared(virtual)
            self.cleared.add(channel)
            self.tracking_states.pop(channel, None)
            self.tracking_wait_rounds[channel] = 0
        else:
            knowledge.observe_clear_failure(waypoint[0], waypoint[1], virtual)
        self._record(channel, 'clear', response['clear_result'], response)
        self._sample_channel(channel)

    def _move_to(self, waypoint):
        waypoint = (float(waypoint[0]), float(waypoint[1]))
        delta_x = waypoint[0] - self.position[0]
        delta_y = waypoint[1] - self.position[1]
        leg = float(np.hypot(delta_x, delta_y))
        if leg > 1e-6:
            # 记录真实发生的移动方位，供折返抑制使用（静止动作不更新）。
            self.last_move_bearing_deg = float(np.degrees(np.arctan2(delta_y, delta_x)))
        self.moved_distance_m += leg
        self.position = waypoint
        self.trajectory.append(waypoint)

    def _record(self, channel, kind, result, response):
        self.steps.append({'kind': kind, 'channel': channel, 'result': result,
                           'x': self.position[0], 'y': self.position[1],
                           'virtual_time_s': float(response['virtual_time_s']),
                           'real_elapsed_s': round(self._real_elapsed_s(), 4)})

    def _sample_channel(self, channel):
        knowledge = self.channels[channel]
        limit = knowledge.max_distance_m(*self.position)
        self.channel_series[channel].append(
            {'measure_count': len(knowledge.observations),
             'limit_m': None if limit is None else float(limit),
             'area_m2': knowledge.possible_area_m2,
             'status': knowledge.status})

    # --------------------------------------------------------- 决策5：结束

    def _inconsistent_channels(self):
        return sorted(channel for channel, knowledge in self.channels.items()
                      if knowledge.is_inconsistent)

    def _exclusion_certificate(self, channel):
        """给出该频道"不存在"结论的可审计依据。

        关键点：掩码为空只说明"按已取得的无信号读数排除完了区域"，真正的严格
        结论还要求这些无信号读数构成**覆盖**——即目标区域内任意位置到某个
        "已取得无信号读数"的停靠点都不超过有效接收半径下界（1000 m）。
        这里把覆盖证据显式算出来写进记录，使"不存在"结论可被复核，
        而不是只看到一个空掩码。

        注意：**已清除频道不适用本证书**（它不是"不存在"，而是"已找到并清除"），
        由 `_exclusion_certificates` 过滤掉。
        """
        knowledge = self.channels[channel]
        silent = [(obs.x, obs.y) for obs in knowledge.observations
                  if obs.result == 'no_signal']
        if not silent:
            return {'no_signal_stations': 0, 'covering_radius_m': None,
                    'receive_radius_lower_bound_m': MIN_RECEIVE_RADIUS_M,
                    'certified': False,
                    'note': '没有无信号读数，无法排除任何位置'}
        stations = np.asarray(silent, dtype=float)
        # 目标圆域上密采样，求到最近"无信号站点"的最大距离。
        # 采样步长约 2.5 m（半径方向）与约 0.5°（角向），足以分辨 10 m 细格网。
        angles = np.linspace(0.0, 2.0 * np.pi, 1441)
        radii = np.linspace(0.0, TARGET_RADIUS_M, 721)
        grid_angles, grid_radii = np.meshgrid(angles, radii)
        samples = np.column_stack((grid_radii.ravel() * np.cos(grid_angles.ravel()),
                                   grid_radii.ravel() * np.sin(grid_angles.ravel())))
        worst = float(np.max(np.min(np.linalg.norm(samples[:, None, :] - stations[None, :, :],
                                                   axis=2), axis=1)))
        return {'no_signal_stations': len(silent),
                'covering_radius_m': round(worst, 3),
                'receive_radius_lower_bound_m': MIN_RECEIVE_RADIUS_M,
                'certified': worst <= MIN_RECEIVE_RADIUS_M}

    def _exclusion_certificates(self):
        """全部被判"不存在"频道的覆盖证明（不含已清除频道），供记录与论文引用。"""
        return {channel: self._exclusion_certificate(channel)
                for channel, knowledge in sorted(self.channels.items())
                if knowledge.status != 'cleared' and knowledge.is_excluded}

    def _finished(self):
        if len(self.cleared) >= MAX_SOURCE_COUNT:
            return 'cleared_limit'
        # §3.6：已发现频道的可能位置集合突然变空属矛盾状态，
        # 先检查约束一致性与数值误差，不得据此宣布该目标不存在。
        if self._inconsistent_channels():
            return 'model_inconsistent'
        if not all(knowledge.is_excluded or knowledge.status == 'cleared'
                   for knowledge in self.channels.values()):
            return None
        # 掩码全空还不够：必须确认"不存在"结论都有覆盖证据（见证书字段）。
        # 若存在未认证的频道，交给覆盖兜底继续补点，而不是就此宣布收工。
        if any(not cert['certified'] for cert in self._exclusion_certificates().values()):
            return None
        return 'all_channels_resolved'

    # ------------------------------------------------------------- 主循环

    def run(self):
        """执行五项决策循环，返回可 JSON 序列化的任务记录。"""
        rounds, stalls = 0, 0
        while True:
            if self.context.should_stop():
                self.stop_reason = 'exit_margin'
                break
            reason = self._finished()
            if reason:
                self.stop_reason = reason
                break
            if rounds >= self.config.max_rounds or stalls >= self.config.stall_limit:
                self.stop_reason = 'stalled'
                break
            rounds += 1
            for channel in list(self.tracking_states):
                if self.channels[channel].is_active:
                    self.tracking_wait_rounds[channel] += 1
            try:
                plan = self._decide()
                if plan is None:
                    # 自救：规划层给出"无动作"不等于任务该结束。若仍有活跃频道，
                    # 用覆盖兜底再试一次（放宽阈值以允许为零星残差跑一趟）；
                    # 兜底也无解时才收尾，并把原因记为 no_action 供人工判读。
                    plan = self._rescue_plan()
                if plan is None:
                    self.stop_reason = ('planner_error' if self.planner_errors else
                                        'no_action')
                    break
                acted = self._execute_measures(plan)
                acted = self._execute_clears(plan.waypoint) or acted
            except BudgetReached:
                self.stop_reason = 'exit_margin'
                break
            stalls = 0 if acted else stalls + 1
        return self._result()

    def _rescue_plan(self):
        """规划层无动作时的自救：放宽兜底阈值，再试一次覆盖补齐。

        常规 `_search_plan` 出于效率考虑会拒绝"为很小的残余专门跑一趟"。
        但当它已经找不到任何动作、而活跃频道仍在时，继续跑一趟的成本
        远低于提前结束（提前结束会留下未证明的频道，可能掩盖真实源）。
        因此这里把阈值放宽到 0，只要还有残余格点就派一个兜底动作。
        """
        if not self._active_channels():
            return None
        relaxed = replace(self.config, backstop_min_hole_m2=0.0)
        original = self.config
        try:
            self.config = relaxed
            return self._backstop_search_plan(
                min(MAX_SOURCE_COUNT - len(self.cleared),
                    CHANNEL_COUNT - len(self.cleared)))
        finally:
            self.config = original

    def _result(self):
        return {
            'stop_reason': self.stop_reason,
            'cleared_channels': sorted(self.cleared),
            'unresolved_channels': sorted(channel for channel, knowledge in self.channels.items()
                                          if knowledge.is_active),
            'inconsistent_channels': self._inconsistent_channels(),
            'start_virtual_time_s': self.start_virtual_time_s,
            'end_virtual_time_s': float(self.context.state.virtual_time_s),
            'real_elapsed_s': self._real_elapsed_s(),
            'moved_distance_m': self.moved_distance_m,
            'actions': len(self.steps),
            'planner_errors': self.planner_errors,
            'steps': self.steps,
            'trajectory': self.trajectory,
            'channel_series': self.channel_series,
            'exclusion_certificates': self._exclusion_certificates(),
            'tracking_states': {
                channel: {
                    'anchor': list(state['anchor']),
                    'bearing_deg': state['bearing_deg'],
                    'upper_m': state['upper_m'],
                    'wait_rounds': self.tracking_wait_rounds[channel],
                }
                for channel, state in sorted(self.tracking_states.items())
            },
            'config': {key: value for key, value in asdict(self.config).items()
                       if not key.startswith('_')},
        }


def run_mission(context, config=None):
    """在给定上下文上运行问题三策略，返回任务记录。

    `config.strategy_mode == 'route'` 时改用 Route-First 策略
    （`problem3_route.RouteFirstStrategy`），正确性层与记录格式完全一致。
    """
    effective = config or Problem3Config()
    if getattr(effective, 'strategy_mode', 'mpc') == 'route':
        from .problem3_route import run_route_mission
        return run_route_mission(context, effective)
    return Problem3Strategy(context, effective).run()


def is_offline_run(context):
    """判断本次会话是否使用本地桩，防止离线结果混入在线成绩目录。"""
    from .offline_stub import OfflineStub
    return isinstance(getattr(context.client, 'transport', None), OfflineStub)


def solve(context):
    """供运行器加载的算法入口，并按在线/离线类型分别保存任务记录。"""
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


def main(argv=None):
    """离线演练入口：本地桩跑随机案例并输出统计量（不连接官方模拟器）。"""
    from .offline_stub import OfflineStub
    from .protocol import RobotClient
    from .scenario import random_scenario
    from .strategy import run_strategy

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sources', type=int, default=12, help='案例干扰源个数')
    parser.add_argument('--seed', type=int, default=1, help='案例随机种子')
    parser.add_argument('--latency', type=float, default=0.0, help='每次请求的附加真实延迟（秒）')
    parser.add_argument('--log', default='output/protocol/offline-p3-mission.jsonl')
    parser.add_argument('--figures', action='store_true', help='输出任务图（需要 matplotlib）')
    args = parser.parse_args(argv)

    scenario = random_scenario(seed=args.seed, n_sources=args.sources)
    stub = OfflineStub(sources=scenario.sources, latency_s=args.latency)
    client = RobotClient('offline-team', stub, log_path=args.log)
    result = run_strategy(client, solve, problem=3)
    print(json.dumps(result['algorithm_result'], ensure_ascii=False, indent=2))
    if args.figures:
        from .problem3_plotting import plot_mission_from_path
        plot_mission_from_path(result['algorithm_result'].get('record_path'),
                               scenario=scenario)
    return result


if __name__ == '__main__':
    main()
