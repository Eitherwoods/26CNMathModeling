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

from .base_models import bearing_deg
from .config import (ANGLE_ERROR_DEG, CHANNEL_COUNT, CHANNEL_SWITCH_TIME_S,
                     CLEAR_RADIUS_M, CLEAR_TIME_S, MAX_RECEIVE_RADIUS_M,
                     MAX_SOURCE_COUNT,
                     MEASURE_TIME_S, MIN_RECEIVE_RADIUS_M, PROBLEM3_OUTPUT_DIR,
                     ROBOT_SPEED_MPS, TARGET_RADIUS_M)
from .problem3_model import ChannelKnowledge, Lattice, apply_direction
from .strategy import BudgetReached

TARGET_AREA_M2 = float(np.pi * TARGET_RADIUS_M ** 2)


@dataclass(frozen=True)
class Problem3Config:
    """问题三数值求解参数。"""

    lattice_spacing_m: float = 10.0
    planning_spacing_m: float = 50.0
    candidate_spacing_m: float = 1200.0
    candidate_radius_m: float = 1500.0
    lookahead_directions: int = 12
    lookahead_lengths_m: tuple = (250.0, 500.0, 900.0)
    lookahead_hypotheses: int = 10
    chase_options_limit: int = 4
    local_lookahead_limit_m: float = 300.0
    approach_direct_m: float = 400.0
    search_channels_per_stop: int = 3
    search_min_gain_m2: float = 5.0e4
    endgame_seconds: float = 60.0
    search_risk_premium: float = 1.25
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
    # 停靠点回访惩罚：抑制收尾阶段在已服务区域之间反复横跳。
    revisit_radius_m: float = 400.0
    revisit_penalty_s: float = 120.0
    # 保证性示向追踪：若已发现频道长期未被服务，则强制执行一次收缩上界的追踪动作。
    tracking_debt_limit_rounds: int = 10


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
        self.stop_reason = 'running'
        self._started_at = time.monotonic()
        self._reach_cache: dict = {}
        # channel -> {anchor: (x, y), bearing_deg: theta, upper_m: U}。
        # 该状态只记录可由示向读数严格保证的追踪上界，不参与细格网可靠结论。
        self.tracking_states: dict = {}
        self.tracking_wait_rounds = {channel: 0 for channel in self.channels}

    def _build_candidates(self):
        """搜索候选停靠点：只保留**落在目标圆域内**的格点。

        `Lattice.build(spacing, radius)` 会把格网扩到 `radius + 覆盖半径`，
        于是半径 1500 的候选集里混进了 (0,±2400)、(±2400,0) 这四个点——
        它们离目标区域边缘 600 m，检测不到任何区域内目标，却会被评分函数
        当成"新增排除面积大"的候选，造成 2400 m 级的无效往返。
        这里按题目给定的目标圆域半径直接滤掉。
        """
        lattice = Lattice.build(self.config.candidate_spacing_m,
                                self.config.candidate_radius_m)
        points = lattice.points
        inside = np.hypot(points[:, 0], points[:, 1]) <= TARGET_RADIUS_M + 1e-9
        return points[inside]

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

    def _unknown_channels(self):
        """返回尚未确认存在、也未清除或排除的频道。"""
        return [channel for channel, knowledge in self.channels.items()
                if knowledge.status == 'unknown' and knowledge.is_active]

    def _confirmed_source_count(self):
        """返回已经由有效读数或清除结果确认存在的频道数。"""
        return sum(knowledge.status in ('detected', 'cleared')
                   for knowledge in self.channels.values())

    def _unknown_source_probability(self):
        """由源总数上下界给出未发现频道的共同存在概率估计。

        这是搜索评分使用的显式均匀先验，不参与排除、可靠清除等严格结论。
        返回值同时包含概率和剩余未知源数的上下界，便于记录与测试。
        """
        unknown_count = len(self._unknown_channels())
        if unknown_count == 0:
            return 0.0, 0, 0
        confirmed = self._confirmed_source_count()
        lower = max(0, 10 - confirmed)
        upper = min(MAX_SOURCE_COUNT - confirmed, unknown_count)
        if upper <= 0:
            return 0.0, lower, upper
        expected = 0.5 * (lower + upper)
        return float(np.clip(expected / unknown_count, 0.0, 1.0)), lower, upper

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
            guaranteed = self._guaranteed_tracking_plan(channel, knowledge)
            if guaranteed is not None:
                options.append(guaranteed)
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

    def _local_lookahead(self, knowledge, limit):
        """细格网局部前瞻：候选点少、假设点取覆盖圆圆心，单次决策保持毫秒级。"""
        center, radius = knowledge.region_estimate(use_fine=True)
        if center is None:
            return None
        step = min(limit, self.config.approach_direct_m)
        angles = np.arange(8) * 45.0
        candidates = [np.asarray(center, dtype=float)]
        for scale in (0.5, 1.0):
            for angle in angles:
                direction = np.array([np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))])
                candidates.append(np.array(self.position) + scale * step * direction)
        candidates.append(np.array(center, dtype=float) + np.array([radius, 0.0]))
        best, best_score = None, float('inf')
        for waypoint in candidates:
            if not self._station_is_new(knowledge.channel, waypoint):
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
            score = travel_s + (total / count) / ROBOT_SPEED_MPS
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
        best, best_score = None, float('inf')
        for length in self.config.lookahead_lengths_m:
            for direction in directions:
                waypoint = np.array(self.position) + length * direction
                if np.hypot(*waypoint) > self.config.candidate_radius_m + 900.0:
                    continue
                if not self._station_is_new(channel, waypoint):
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
                score = travel_s + (total / count) / ROBOT_SPEED_MPS
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

        三段式：
        1. 按新增排除面积下限筛选（常规高增益搜索）；
        2. 去掉下限再找一次（只剩零星残差时的常规兜底）；
        3. **覆盖兜底**：若仍有整块残余空洞没被候选格网命中，直接以
           "补齐最大连通空洞"为目标选点。

        关于覆盖完备性（`verify_coverage_completeness` 可离线验证）：
        固定候选格网中在目标圆域内的 9 个点
        （中心 + (±1200,0) + (0,±1200) + 四个对角点 (r=1697)）
        对任意源位置的最坏距离为 **848.36 m**，小于有效接收半径下界 1000 m，
        因此**若某频道在这 9 点全部无信号，则该频道确实不存在**——
        这是一条严格结论，不依赖轨迹偶然性。

        第 3 段兜底仍然保留，理由是"9 点全测"要求该频道在每个点都被安排检测，
        而每站只取 `search_channels_per_stop` 个未发现频道；当活跃频道很多、
        时间预算又紧时，`_search_cap` 会进一步收紧每站检测数，理论上存在
        "某频道凑不齐 9 点就被判终止"的窗口。兜底段是这一窗口的安全网，
        同时也能在候选格网因配置改动而不再完备时自动补救。
        """
        # 剩余源数的上界：既受总数上限约束，也不能超过"还没清除的频道数"。
        # 注意不能用 len(_active_channels())——它包含尚未判定排除的频道，
        # 会把剩余源数估高，进而高估搜索性价比、低估追击优先级。
        uncleared = CHANNEL_COUNT - len(self.cleared)
        remaining_sources = min(MAX_SOURCE_COUNT - len(self.cleared), uncleared)
        if remaining_sources <= 0:
            return None
        for min_gain in (self.config.search_min_gain_m2, 0.0):
            plan = self._best_search_plan(remaining_sources, min_gain)
            if plan is not None:
                return plan
        return self._backstop_search_plan(remaining_sources)

    def verify_coverage_completeness(self, margin_m=0.0):
        """离线自检：候选停靠点是否足以对任意源位置给出"无信号"或"发现"。

        返回 (完备?, 最坏距离m, 依据)。
        判据：目标圆域内任意点到最近候选点的距离 <= MIN_RECEIVE_RADIUS_M 时，
        该点的源在任何一次检测中都不可能被漏过（有效接收半径 >= 1000 m）。
        因此"九点全测仍无信号"即为严格的不存在性证明。
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
        # 候选：空洞的采样点 + 空洞里离当前位置最近的若干点。
        sample = hole[::max(1, len(hole) // 48)]
        nearest = hole[np.argsort(np.linalg.norm(hole - np.array(self.position), axis=1))[:4]]
        # 再沿当前位置到空洞重心的方向补几个中间点，避免"一步到位"式长跳：
        # 中间点让策略有机会在通往空洞的路上顺带取得读数。
        centroid = hole.mean(axis=0)
        position = np.asarray(self.position, dtype=float)
        bridges = []
        for ratio in (0.35, 0.7):
            bridges.append(position + ratio * (centroid - position))
        candidates = np.vstack((sample, nearest, np.asarray(bridges)))
        best, best_score = None, float('inf')
        for waypoint in candidates:
            measures = self._search_options(waypoint, 0.0, None)
            if not measures:
                continue
            measures = measures[:max(1, self._search_cap())]
            gain = self._measure_gain(measures, waypoint)
            if gain <= 0.0:
                continue
            expected = self._expected_finds(measures, waypoint)
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
            measures = self._search_options(waypoint, min_gain, None)
            if not measures:
                continue
            measures = measures[:max(1, self._search_cap())]
            gain = self._measure_gain(measures, waypoint)
            expected = self._expected_finds(measures, waypoint)
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

    def _expected_finds(self, channels, waypoint):
        """逐频道估计本次扫描能新发现的源数。

        已发现频道只计检测成本和后验收紧价值，不再重复计作“新发现”；未发现频道
        的存在概率由当前已确认源数及 10--16 个总数约束给出，空间命中概率则按该
        频道当前可能区域中被停靠点可靠接收圆覆盖的比例计算。
        """
        source_probability, _, _ = self._unknown_source_probability()
        if source_probability <= 0.0:
            return 0.0
        expected = 0.0
        for channel in channels:
            knowledge = self.channels[channel]
            if knowledge.status != 'unknown' or not knowledge.is_active:
                continue
            total_area = knowledge.possible_area_m2
            if total_area <= 0.0:
                continue
            hit_probability = min(1.0, self._search_gain(channel, waypoint) / total_area)
            expected += source_probability * hit_probability
        return expected

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
            if (wait_rounds < self.config.tracking_debt_limit_rounds
                    or channel in represented
                    or not self.channels[channel].is_active):
                continue
            guaranteed = self._guaranteed_tracking_plan(channel, self.channels[channel])
            if guaranteed is not None:
                options.append(guaranteed)
        options = [plan for plan in options if np.isfinite(plan.score_s)]
        if not options:
            return None
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
        return options[0]

    # --------------------------------------------------------- 决策3/4：执行

    def _execute_measures(self, plan):
        acted = False
        x, y = float(plan.waypoint[0]), float(plan.waypoint[1])
        for channel in self._order_measures(plan.measures):
            if self.context.should_stop():
                break
            if not self._station_is_new(channel, plan.waypoint):
                continue
            response = self.context.measure(x, y, channel)
            guaranteed_upper = (
                plan.guaranteed_upper_m
                if plan.kind == 'guaranteed_track' and channel == plan.target_channel
                else None)
            self._apply_measure(
                channel, plan.waypoint, response,
                guaranteed_upper_m=guaranteed_upper)
            acted = True
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
        self.current_channel = channel
        self._record(channel, 'measure', result, response)
        self._sample_channel(channel)

    def _execute_clears(self, waypoint):
        acted = False
        x, y = float(waypoint[0]), float(waypoint[1])
        for channel in self._clear_candidates(waypoint):
            if self.context.should_stop():
                break
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
        self.moved_distance_m += float(np.hypot(waypoint[0] - self.position[0],
                                                waypoint[1] - self.position[1]))
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
    """在给定上下文上运行问题三策略，返回任务记录。"""
    return Problem3Strategy(context, config).run()


def solve(context):
    """供运行器加载的算法入口：执行策略并回报汇总（记录写盘失败不影响任务）。"""
    record = run_mission(context)
    summary = summarize(record)
    try:
        PROBLEM3_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d-%H%M%S')
        target = PROBLEM3_OUTPUT_DIR / f'mission_p{context.problem}_{stamp}.json'
        target.write_text(json.dumps({'summary': summary, 'record': record},
                                     ensure_ascii=False, indent=2), encoding='utf-8')
        summary['record_path'] = str(target)
    except OSError:
        summary['record_path'] = None
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
