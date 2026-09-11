# output/protocol — 协议日志与离线自检记录

本目录放两类东西：

1. **原始协议日志**（`.jsonl`）：与模拟器（或离线桩）之间逐条收发的请求/响应流，一条一行。
   离线与演练都写这里，靠文件名前缀区分。
2. **离线自检的任务记录**（`offline-mission_*.json`）：离线桩跑完一局的汇总与逐步轨迹。

> **在线（模拟器演练 / 正式测试）的任务记录不在这里**，它们在 `output/Problem3`、`output/Problem4`，
> 格式是策略写出的 `.json` 加界面导出的 `.txt` 动作记录。见 `output/README.md`。
> 这样分的理由：离线桩不复刻官方误差场与判定细节，它的数字**不是成绩**，
> 必须和演练结果物理隔开，免得自检数据被误当演练成绩引用。

## 命名规则

| 前缀 | 含义 | 示例 |
| --- | --- | --- |
| `offline-*.jsonl` | 离线桩的协议日志 | `offline-p4.jsonl` |
| `practice-*.jsonl` | 人工触发的模拟器演练的协议日志（**不是成绩**，成绩看 `output/ProblemN`） | `practice-p4-01.jsonl` |
| `offline-mission_*.json` | 离线自检的任务记录 | `offline-mission_p4_20260911-142326.json` |
| `test-*.jsonl` | 单元测试固定写入的日志，每次跑测试会被覆盖 | `test-problem3-offline.jsonl` |

代码侧由 `codes/config.py:record_dir_for()` 决定落盘目录，`codes/problem4_solution.py:solve()`
在发令结束后按"是否跑在 `OfflineStub` 上"自动选目录并加 `offline-` 前缀，不需要人工搬。

## 现有文件

### 问题四离线自检记录

由 `python -m codes.problem4_solution --sources N --seed S [--directional K]` 生成；
同一命令加 `--figures` 会顺手出四联图（图在 `figures/Problem4/`）。

| 文件 | 案例 | 虚拟时间 | 动作 | 清除 | 收尾 |
| --- | --- | --- | --- | --- | --- |
| `offline-mission_p4_20260911-135016.json` | 12 源（6 定向），**优化前**调度 | 40008.2 s | 639 | 12/12 | `all_channels_resolved` |
| `offline-mission_p4_20260911-135134.json` | 16 源全向，**优化前**调度 | 44966.0 s | 669 | 16/16 | `cleared_limit` |
| `offline-mission_p4_20260911-142326.json` | 12 源（6 定向），**优化后**调度 | 17825.9 s | 609 | 12/12 | `all_channels_resolved` |
| `offline-mission_p4_20260911-142336.json` | 16 源全向，**优化后**调度 | 20799.3 s | 636 | 16/16 | `cleared_limit` |

前后两组对照即"虚拟时间调度优化 −50.6%"的来源，逐档扫描表见 `solutions/problem4_flow.md`
的"优化目标：虚拟时间"一节。**这四份都是本地桩数字，不能当成绩。**

### 协议日志

| 文件 | 行数 | 来源 |
| --- | --- | --- |
| `offline-p3.jsonl` | 48 | `run_robot --mode offline --problem 3` 的默认日志 |
| `offline-demo.jsonl` | 12 | 演示策略 `strategy_demo` 的离线日志 |

**已知的小漂移**：`codes/problem3_solution.py` 的 `main()` CLI 默认 `--log` 是历史命名
`output/protocol/offline-p3-mission.jsonl`（从没人写过这个文件）；`run_robot` 的默认
才是 `offline-p3.jsonl`。两条路径互不覆盖，仅是命名不一致。按"先不影响问题三代码"的
约束暂未统一，要并轨只需改 p3 `main()` 的 default 字符串（不影响策略逻辑与测试）。
| `practice-p3-01.jsonl` | 2718 | `tester/runq3.txt` 的真实演练；5 段会话 / 1343 动作 / 157.4 s 墙钟，**"117 ms/动作"这个实测结论的原始依据** |
| `practice-p3-demo.jsonl` | 22 | `codes/README.md` 里的演练走查命令 |
| `practice-p4-01.jsonl` | 1204 | `tester/runq4.txt` 的真实演练（14:15 那局，10 源 10/10 清除） |
| `test-problem3-offline.jsonl` | 2418 | `codes/test_problem3.py` 用例固定写入，每次跑测试重写 |

## 常用命令

```powershell
# 离线自检（记录自动落到本目录的 offline-mission_p4_*.json）
python -m codes.problem4_solution --sources 12 --seed 1 --figures

# 从已有记录重新汇总 / 重新出图
python -m codes.problem4_solution output/protocol/offline-mission_p4_20260911-142326.json
python -m codes.problem4_plotting output/protocol/offline-mission_p4_20260911-142326.json --name my_run

# 演练（必须人工确认模拟器已在演练模式；不要点正式测试）
python -m codes.run_robot --mode practice --problem 4 --strategy codes.strategy_p4:solve --confirm-practice --log output/protocol/practice-p4-01.jsonl
```
