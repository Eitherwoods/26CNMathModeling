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
from dataclasses import asdict, dataclass, field

import numpy as np

from .base_models import bearing_deg
from .config import (ANGLE_ERROR_DEG, CHANNEL_COUNT, CHANNEL_SWITCH_TIME_S,
                     CLEAR_RADIUS_M, CLEAR_TIME_S, MAX_SOURCE_COUNT,
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
        self.candidates = Lattice.build(self.config.candidate_spacing_m,
                                        self.config.candidate_radius_m).points
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
            if limit <= CLEAR_RADIUS_M:
                options.append(StopPlan('clear', np.array(self.position), [], 0.0, 0.0,
                                        note=f'频道{channel}已可证清除'))
                continue
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
                                    score_s=score, note=note))
        return options

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

        先按新增排除面积下限筛选；若全区都找不到达标候选（只剩零星残差），
        再去掉下限再找一次，避免因残差无法排除而无法给出完整结论。
        """
        remaining_sources = min(MAX_SOURCE_COUNT - len(self.cleared), len(self._active_channels()))
        if remaining_sources <= 0:
            return None
        for min_gain in (self.config.search_min_gain_m2, 0.0):
            plan = self._best_search_plan(remaining_sources, min_gain)
            if plan is not None:
                return plan
        return None

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
            expected = self._expected_finds(remaining_sources, gain)
            if expected <= 0.0:
                continue
            # 新发现干扰源的后续清除代价也要计入，否则会高估搜索的性价比。
            follow_up = self._follow_up_seconds(waypoint) + self.config.endgame_seconds
            score = (self._travel_s(waypoint) + self._measure_cost_s(measures)) / expected + follow_up
            score *= self.config.search_risk_premium
            if score < best_score:
                best = StopPlan('sweep', waypoint, measures, self._travel_m(waypoint),
                                score_s=score, expect_finds=expected)
                best_score = score
        return best

    def _candidate_waypoints(self):
        """搜索候选停靠点：当前点与固定候选格网。"""
        return np.vstack((np.asarray(self.position, dtype=float), self.candidates))

    def _expected_finds(self, remaining_sources, gain_m2):
        """均匀先验下的期望发现数：每个频道持源概率为剩余源数 / 频道数。"""
        if gain_m2 <= 0.0:
            return 0.0
        return remaining_sources / CHANNEL_COUNT * gain_m2 / TARGET_AREA_M2

    def _follow_up_seconds(self, waypoint):
        """在 waypoint 新发现一个目标后，还需要多少移动时间（用可能区域估计）。"""
        mask = np.ones(len(self.coarse.points), dtype=bool)
        remaining = np.linalg.norm(self.coarse.points[mask] - waypoint, axis=1)
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
        options = [plan for plan in options if np.isfinite(plan.score_s)]
        if not options:
            return None
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
            self._apply_measure(channel, plan.waypoint, response)
            acted = True
        return acted

    def _apply_measure(self, channel, waypoint, response):
        self._move_to(waypoint)
        knowledge = self.channels[channel]
        result = response['measure_result']
        virtual = float(response['virtual_time_s'])
        if result == 'direction':
            knowledge.observe_direction(waypoint[0], waypoint[1], float(response['svd_deg']), virtual)
        elif result == 'near':
            knowledge.observe_near(waypoint[0], waypoint[1], virtual)
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
        for channel in self._detected_channels():
            knowledge = self.channels[channel]
            if knowledge.cleared_here(waypoint[0], waypoint[1]):
                continue
            if (knowledge.certain_clear(waypoint[0], waypoint[1])
                    or knowledge.near_certain_clear(waypoint[0], waypoint[1])):
                certain.append(channel)
                continue
            limit = knowledge.max_distance_m(waypoint[0], waypoint[1])
            probability = knowledge.clear_probability(waypoint[0], waypoint[1])
            if (limit is not None and limit <= self.config.speculative_clear_m
                    and probability >= self.config.speculative_min_probability):
                speculative.append((probability, channel))
        speculative.sort(key=lambda item: -item[0])
        return certain + [channel for _, channel in speculative]

    def _apply_clear(self, channel, waypoint, response):
        self._move_to(waypoint)
        knowledge = self.channels[channel]
        virtual = float(response['virtual_time_s'])
        if response['clear_result'] == 'success':
            knowledge.mark_cleared(virtual)
            self.cleared.add(channel)
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

    def _finished(self):
        if len(self.cleared) >= MAX_SOURCE_COUNT:
            return 'cleared_limit'
        if all(not knowledge.is_active for knowledge in self.channels.values()):
            return 'all_channels_resolved'
        return None

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
            try:
                plan = self._decide()
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

    def _result(self):
        return {
            'stop_reason': self.stop_reason,
            'cleared_channels': sorted(self.cleared),
            'start_virtual_time_s': self.start_virtual_time_s,
            'end_virtual_time_s': float(self.context.state.virtual_time_s),
            'real_elapsed_s': self._real_elapsed_s(),
            'moved_distance_m': self.moved_distance_m,
            'actions': len(self.steps),
            'planner_errors': self.planner_errors,
            'steps': self.steps,
            'trajectory': self.trajectory,
            'channel_series': self.channel_series,
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
