# q3/q4 迭代式联调最终报告

日期：2026-09-12

## 运行口径

- 基准引擎：`codes.benchmark_q34_local` 的本地官方 Engine，固定 seed，串行执行。
- 主指标：每个案例的 `virtual_time_s / cleared`，再对案例取平均；所有约束由 Engine 重放审计。
- 完成判据：清除集合与真值完全一致，轨迹、动作时间、频道范围、坐标边界、清除反馈和 `planner_errors` 全部通过。
- 用户提到的单干扰源约 200 s 记录未在仓库中找到对应 seed/命令，因此没有把它和 10–16 源官方演练生成器混合比较。

## 固化修改

- `codes/config.py`
  - 新增 q3/q4 追踪策略默认参数集中配置。
  - q3 默认 `adaptive_initial_direction=True`、`tracking_path_limit_m=500.0`。
  - q4 默认 `allocation_by_plan=True`、`tracking_path_limit_m=900.0`；q4 的自适应初始方向仍关闭。
- `codes/problem3_solution.py`
  - 复用现有自适应方向和路径限制实现，由配置集中值驱动默认行为。
- `codes/problem4_solution.py`
  - 复用现有计划级频道分配和路径限制实现，由配置集中值驱动默认行为。

## 逐轮实验

| 轮次 | 变化 | 案例 | 平均每源虚拟时间 | 完成/审计 |
| --- | --- | ---: | ---: | --- |
| B3 | q3 当前基线 | seeds 2,3 | 351.13 s | 2/2 |
| Q3-R1 | 仅自适应初始方向 | seeds 2,3 | 308.65 s | 2/2 |
| Q3-R2 | 在 R1 上增加 500 m 追踪步长限制 | seeds 2,3 | 304.68 s | 2/2 |
| Q3-H | 串行 holdout 基线 | seeds 11,12,13 | 387.97 s | 3/3 |
| Q3-H2 | 串行 holdout R2 | seeds 11,12,13 | 368.48 s | 3/3 |
| B4 | q4 当前基线 | seeds 2,3 | 641.04 s | 2/2 |
| Q4-R1 | 仅计划级频道分配 | seeds 2,3 | 632.12 s | 2/2 |
| Q4-R2 | 仅 900 m 追踪步长限制 | seeds 2,3 | 637.53 s | 2/2 |
| Q4-R3 | R1 + R2 | seeds 2,3 | 628.75 s | 2/2 |
| Q4-H | q4 当前基线 | seeds 11–15 | 683.53 s | 5/5 |
| Q4-H2 | R1 + R2 | seeds 11–15 | 658.36 s | 5/5 |
| Q4-U | 未参与调参的默认配置复核 | seeds 4–6 | 657.12 s | 3/3 |
| Q4-U-B | 同组基线 | seeds 4–6 | 656.21 s | 3/3 |

q3 串行 holdout 相对基线下降约 5.0%；q4 五案例 holdout 相对基线下降约 3.7%。q4 seeds 4–6 的每源均值基本持平，说明收益存在案例依赖，不能承诺每一局都下降。

## 采用与放弃

- 采用：q3 自适应初始方向；它把方向采样从固定东向环改为“当前位置到当前可行域中心”的旋转环，避免初始方向与残余区域错位。
- 采用：q3 500 m、q4 900 m 追踪路径限制；仅限制追踪候选的单腿长度，不改变搜索证书、可行集或可靠清除判据。
- 采用：q4 计划级频道分配；在公平年龄约束下，对候选频道的“移动 + 预期剩余定位”统一评分。
- 放弃默认：q4 自适应初始方向。它在混合方向案例的多轮对照中变差，保留为显式参数以便消融。
- 放弃默认：q3 路线承诺式巡游、q4 多起点路线、机会式搜索、连续证据共享、LLS 试探清除等方案；已有归档或当前复验显示收益不稳定，且会增加动作数或墙钟。

## 风险与边界

- 评价使用本地 Engine；正式模拟器还应按 `tester/runq3.txt`、`tester/runq4.txt` 在演练模式复核。
- q3 前瞻和 q4 计划分配会增加计算量；当前本地 10–16 源案例仍远低于 1200 s 真实窗口，但正式演练需继续观察墙钟。
- q4 seeds 4–6 的每源均值略高于同组基线，说明参数不是全局最优；若正式局分布明显偏离当前生成器，应优先回退 `allocation_by_plan=False`，保留 900 m 路径限制。
- 所有已跑案例均未出现空解、非法频道、轨迹断裂、边界越界、清除真值不符、数值非有限或 `planner_errors`。

## 可复现实验文件

本轮最新测试结果保留在：

- `round_summary.md`、`round_summary.json`
- `round_correctness_20260912.log`
- `round1_q3_baseline_23.json`
- `round2_q3_optimized_23.json`
- `round3_q4_baseline_23.json`
- `round4_q4_optimized_23.json`
- `round5_q4_optimized_holdout_1115.json`
- `final_serial_p3_baseline_111213.json`
- `final_serial_p3_candidate_111213.json`

旧的逐轮扫档原始 JSON 不参与运行，已清理；需要时可由 `codes.benchmark_q34_local` 重新生成。

正确性回归：q3/q4/共享优化测试共 176 项，171 项通过、5 项按设计跳过；`python -m compileall -q codes` 通过。
