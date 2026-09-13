# -*- coding: utf-8 -*-
"""问题三 Route-First 策略：全局清除巡游 + 两站交会定位。

与 MPC 策略（`Problem3Strategy`）的差别
----------------------------------------
MPC 每轮在"追击"与"搜索"之间按期望耗时择优，收敛段需要 5-7 次示向逐次收缩。
本策略把动作倒过来：

1. **全局巡游线**：对固定站点集（hybrid15 或证书九点）预排最近邻 + 2-opt
   巡游路线，机器狗按线行进；每站批量测量全部正增益频道——发现与空频道
   的"不存在证明"都是巡游的副产品，不再有专属搜索巡航。
2. **两站交会替代逐次收敛**：两个示向度（基线 s、源距 d）的交会不确定半径
   约 ε·d²/s；d=1200、s=800 时约 31 m。掩码最小覆盖圆半径一旦小于
   `route_spec_radius_m`，直接偏离巡游去圆心测量并清除——每源 2-3 次测量。
3. **受控偏离**：只有交会圆心进入偏离预算（`route_deviation_budget_m`）才
   离线清除，否则继续巡游，由后续站点补第二次示向。

正确性层（细格网掩码、覆盖证书、`_finished` 认证门禁、双时钟预算）全部
继承自 `Problem3Strategy`，本类只替换决策核心 `_decide`。巡游耗尽或停滞时
回退到继承的 MPC 决策，保证任务总能收尾。
"""
from __future__ import annotations

import numpy as np

from .config import TARGET_RADIUS_M
from .problem3_model import Lattice
from .problem3_solution import Problem3Strategy, StopPlan


class RouteFirstStrategy(Problem3Strategy):
    """巡游主导的问题三策略：按预排路线测量，交会后受控偏离清除。"""

    def __init__(self, context, config=None):
        super().__init__(context, config)
        self.tour_stations = self._build_tour_stations()
        self.tour_queue = self._plan_tour()
        self.tour_index = 0

    # ------------------------------------------------------------- 巡游构造

    def _build_tour_stations(self):
        """巡游站点集：'grid9' 只走证书九点；默认走全部候选（hybrid15）。"""
        if self.config.route_tour_set == 'grid9':
            lattice = Lattice.build(self.config.candidate_spacing_m,
                                    self.config.candidate_radius_m)
            points = lattice.points
            inside = np.hypot(points[:, 0], points[:, 1]) <= TARGET_RADIUS_M + 1e-9
            return points[inside]
        return np.asarray(self.candidates, dtype=float)

    def _plan_tour(self):
        """最近邻 + 2-opt 预排巡游顺序（纯排序，不改站点集合）。"""
        points = [np.asarray(point, dtype=float) for point in self.tour_stations]
        remaining = list(range(len(points)))
        order = []
        position = np.asarray(self.position, dtype=float)
        while remaining:
            nearest = min(remaining,
                          key=lambda index: float(np.linalg.norm(points[index] - position)))
            order.append(nearest)
            remaining.remove(nearest)
            position = points[nearest]

        def path_length(sequence):
            previous = np.asarray(self.position, dtype=float)
            total = 0.0
            for index in sequence:
                total += float(np.linalg.norm(points[index] - previous))
                previous = points[index]
            return total

        improved = True
        while improved:
            improved = False
            for i in range(len(order) - 1):
                for j in range(i + 1, len(order)):
                    candidate = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                    if path_length(candidate) < path_length(order) - 1e-6:
                        order = candidate
                        improved = True
        return [points[index] for index in order]

    # ------------------------------------------------------------- 决策核心

    def _decide(self):
        """优先级：可清除目标受控偏离 > 巡游下一站 > 回退 MPC。

        `route_handoff_detected` 开启时，存在"已发现但交会未成熟"的频道
        就把本轮交给 MPC 追击（巡游的批量发现 + MPC 的收敛各取所长）；
        清完回到巡游线继续发现与补证。
        """
        try:
            deviation = self._deviation_plan()
            if deviation is not None:
                return deviation
            if (self.config.route_handoff_detected
                    and self._detected_channels()):
                return super()._decide()
            if (self.config.route_max_nodes > 0
                    and self.tour_index >= self.config.route_max_nodes):
                return super()._decide()
            node_plan = self._next_node_plan()
            if node_plan is not None:
                # 长途巡游腿复用 MPC 的中途检测点机制（顺路测量加速发现）。
                return self._maybe_intermediate_plan(node_plan) or node_plan
        except Exception:
            self.planner_errors += 1
        # 巡游耗尽或规划异常：交给继承的 MPC 决策收尾（追击残差 + 补证）。
        return super()._decide()

    def _deviation_plan(self):
        """交会成熟目标的受控偏离清除计划；无成熟目标返回 None。"""
        best = None
        best_radius = float('inf')
        for channel in self._observed_channels():
            knowledge = self.channels[channel]
            center, radius = knowledge.region_estimate(use_fine=True)
            if center is None or radius > self.config.route_spec_radius_m:
                continue
            center = np.asarray(center, dtype=float)
            if float(np.linalg.norm(center - np.asarray(self.position))) \
                    > self.config.route_deviation_budget_m:
                continue
            if radius < best_radius:
                best_radius = radius
                best = (channel, center, radius)
        if best is None:
            return None
        channel, center, radius = best
        knowledge = self.channels[channel]
        waypoint = center
        if not self._station_is_new(channel, waypoint):
            # 圆心已测过：稍微偏移一点取新读数，进一步收缩掩码。
            offset = np.array([25.0, 0.0]) if center[0] <= 0 else np.array([-25.0, 0.0])
            waypoint = center + offset
        measures = [channel] if self._station_is_new(channel, waypoint) else []
        score = (self._travel_s(waypoint) + self._measure_cost_s(measures)
                 + self.config.endgame_seconds)
        return StopPlan(
            'route_deviate', waypoint, measures, self._travel_m(waypoint),
            score_s=score,
            note=f'交会偏离：频道{channel} 不确定半径 {radius:.0f} m')

    def _next_node_plan(self):
        """巡游下一站的批量测量计划；跳过无增益站点。"""
        while self.tour_index < len(self.tour_queue):
            node = self.tour_queue[self.tour_index]
            measures = self._search_options(node, 0.0, None)
            if measures:
                return StopPlan(
                    'route_node', np.asarray(node, dtype=float), measures,
                    self._travel_m(node), score_s=self._travel_s(node),
                    note=f'巡游站 {self.tour_index + 1}/{len(self.tour_queue)}')
            # 该站对所有频道都无正增益：跳过（证书也不需要这里的无信号读数）。
            self.tour_index += 1
        return None

    def _execute_measures(self, plan):
        acted = super()._execute_measures(plan)
        if plan.kind == 'route_node' and self.tour_index < len(self.tour_queue) \
                and np.allclose(np.asarray(plan.waypoint, dtype=float),
                                np.asarray(self.tour_queue[self.tour_index],
                                           dtype=float), atol=1e-6):
            self.tour_index += 1
        if plan.kind == 'route_deviate':
            # 偏离清除后，从当前位置对剩余站点重新最近邻排序，
            # 避免按原顺序走到远端节点（偏离常把机器人带离原巡游线）。
            remaining = self.tour_queue[self.tour_index:]
            if len(remaining) > 1:
                position = np.asarray(self.position, dtype=float)
                pending = list(remaining)
                reordered = []
                while pending:
                    nearest = min(
                        range(len(pending)),
                        key=lambda k: float(np.linalg.norm(
                            np.asarray(pending[k], dtype=float) - position)))
                    position = np.asarray(pending[nearest], dtype=float)
                    reordered.append(pending.pop(nearest))
                self.tour_queue = reordered
                self.tour_index = 0
        return acted


def run_route_mission(context, config=None):
    """在给定上下文上运行 Route-First 策略，返回任务记录。"""
    return RouteFirstStrategy(context, config).run()
