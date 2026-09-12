# 2026 CUMCM 国赛 B 题：无线电干扰源定位与清除

<<<<<<< HEAD
本项目为 2026 年全国大学生数学建模竞赛 B 题的完整参赛工程，包含四个问题的模型与求解代码、与官方模拟器的 HTTP 协议实现、离线自检与演练自动化工具、论文图件与 LaTeX 正文。
=======
本项目为 2026 年全国大学生数学建模竞赛 B 题的完整参赛工程，包含四个问题的模型与求解代码、与官方模拟器的 HTTP 协议实现、正确性单元测试与演练自动化工具、论文图件与 LaTeX 正文。
>>>>>>> b15e9cf5f3b345ae6a2e9132e036edb8871ea08f

**主线模型**：以"动态检测与清除寻路算法"贯穿问题三、四——问题一建立角域交会与最小定位区域算法，问题二建立纯几何检测价值与第二检测点求解，问题三在源数未知的条件下引入行进距离与候选点集的精确最短路线，问题四扩展至定向源（联合位置×类型×接收半径×朝向的保守外包）。总建模方案见 [solutions/modeling-strategy.md](solutions/modeling-strategy.md)。

## 目录结构

| 目录 | 内容 |
|---|---|
| `codes/` | 全部 Python 代码：几何模型、四问求解器、协议层、离线桩、演练自动化、测试（详见 [codes/README.md](codes/README.md)、[codes/README_problem1.md](codes/README_problem1.md)） |
| `solutions/` | 建模方案与流程文档：总方案、问题 2/3/4 分问流程、审查记录（`REVIEW-*.md`）、题目原文（`Question-B/`） |
| `body/` | 论文 LaTeX 工程（`main.tex` + `cumcmthesis.cls`），编译产物 `main.pdf` |
| `figures/` | 论文插图，按问题分目录，PNG 预览 + SVG 论文版（总索引见 [figures/README.md](figures/README.md)） |
<<<<<<< HEAD
| `output/` | 运行结果：离线自检进 `output/protocol/`，在线演练/正式记录进 `output/Problem3|4/`（约定见 [output/README.md](output/README.md)） |
=======
| `output/` | 运行结果：演练协议日志进 `output/protocol/`，在线演练任务记录进 `output/Problem3\|4/`（约定见 [output/README.md](output/README.md)） |
>>>>>>> b15e9cf5f3b345ae6a2e9132e036edb8871ea08f
| `tester/` | 官方模拟器 `jammers-simulator.exe`、演练命令（`runq3.txt`/`runq4.txt`）、演练自动化说明、模拟器数据 |

## 环境要求

- Python 3.9+，依赖 NumPy、SciPy、Matplotlib（已验证于本机 `E:\Python` 环境），协议层无第三方依赖。
- 所有命令一律在**项目根目录**执行，不要先 `cd` 进 `codes`。
- Windows 下建议加 `-X utf8` 避免控制台编码问题。

## 快速开始

### 问题一：交会区域直径

```powershell
python -X utf8 -m codes.problem1_solution          # 五点实例 + 正三角形反例，输出 output/Problem1/
python -X utf8 -m unittest codes.test_problem1 -v
```

### 问题二：第二检测点

不依赖模拟器，以第一检测点坐标和示向度为输入：

```powershell
python -m codes.problem2_solution --station 0 0 --bearing 0
python -m unittest codes.test_problem2 -v
```

结果写入 `output/Problem2/problem2_results.json`，候选区域图写入 `figures/Problem2/`。

<<<<<<< HEAD
### 问题三 / 问题四：离线自检

两问均不依赖官方模拟器即可自检（离线桩 `offline_stub.py` + 本地案例工厂）：

```powershell
python -m unittest codes.test_problem3 -v
python -m codes.problem3_solution --seed 7 --sources 12 --figures

python -m unittest codes.test_problem4 -v                # 49 项（5 项端到端默认跳过）
$env:RUN_PROBLEM4_E2E = '1'                              # 需要端到端时显式开启
python -m codes.problem4_solution --sources 12 --seed 1 --figures
```

**注意**：离线统计量不是官方演练成绩，仅验证"模型 + 策略 + 桩 + 绘图"链路可用。
=======
### 问题三 / 问题四：本地正确性自检（只做这一步，然后直接上演练）

**测试策略（2026-09-12 起）**：性能与参数评价一律以**服务器"演练模式"实测为准**；本地只保留下面两套单元测试验证代码正确性，通过后直接跑线上演练，不再做本地离线仿真、基准或扫档。历史离线测试记录已于 2026-09-12 彻底删除，`offline-*`、`test-*.jsonl` 等本地测试产物已加入 `.gitignore`，不再入库。

```powershell
python -m unittest codes.test_problem3 -v
python -m unittest codes.test_problem4 -v    # 5 项端到端默认跳过，$env:RUN_PROBLEM4_E2E='1' 显式开启
```

**注意**：离线桩只是协议测试替身，其统计量不是官方演练成绩；`problem3/4_solution` 的离线 CLI 仅作故障排查用，不再作为常规测试流程，其记录不再保留。
>>>>>>> b15e9cf5f3b345ae6a2e9132e036edb8871ea08f

### 连接官方模拟器进行演练

1. 打开 `tester/jammers-simulator.exe` 并登录，确认机器狗接口端口（默认 2026）。
2. 在模拟器中选择"**问题 N 演练测试**"，等待接口就绪。
   **严禁触碰任何"正式测试"入口。**
3. 从项目根目录运行（见 `tester/runq3.txt`、`tester/runq4.txt`）：

```powershell
python -m codes.run_robot --mode practice --problem 3 --strategy codes.strategy_p3:solve --confirm-practice --log output/protocol/practice-p3-01.jsonl
python -m codes.run_robot --mode practice --problem 4 --strategy codes.strategy_p4:solve --confirm-practice --log output/protocol/practice-p4-01.jsonl
```

`robot_id` 由 `tester/username-and-password.txt` 的"参赛队号"字段自动读取，密码不进入任何请求、日志或命令行。详细步骤、故障排查与协议约束见 [codes/README.md](codes/README.md) 的"打开软件后，如何进行HTTP演练"一节。

演练结束后可从任务记录 JSON 直接重画问题四四联图：

```powershell
python -m codes.problem4_plotting "output/Problem4/mission_p4_时间戳.json" --name practice_p4_L2
```

### 演练 GUI 自动化

```powershell
python -m codes.gui_autopilot
```

自动完成：代启模拟器 → 登录 → 关闭公告 → 点击"开始问题 N 演练测试"（白名单逐字匹配）→ 等接口就绪 → 跑 practice 命令 → 自动生成 TXT。对含"正式"的按钮只记录不点击，正式测试入口硬编码禁触。技术要点见 [tester/README.md](tester/README.md)。

## 代码架构

```
run_robot.py（入口，默认离线）
  ├─ protocol.py        RobotClient / RobotState / HttpTransport（显式开启才联网）
  ├─ offline_stub.py    确定性离线响应桩（串行、幂等缓存、计时）
  └─ strategy_pN.py     solve(context) 统一算法入口
        ├─ problem3_solution.py + problem3_model.py + problem3_route.py
        └─ problem4_solution.py + problem4_model.py
```

协议层四个接口：`enter()`、`measure(x,y,channel)`、`clear(x,y,channel)`、`exit()`。
算法通过 `context.measure/clear/state/should_stop()` 与模拟器交互，不自行发送 HTTP、不调用 enter/exit。接入方式与接口表详见 [codes/README.md](codes/README.md)。

<<<<<<< HEAD
各问题测试文件：`test_problem1~4.py`、`test_protocol.py`（计时、丢包、拒绝、超时、边界与 HTTP 传输检查）、`test_strategy.py`、`benchmark_problem3/4.py`（性能基准）。
=======
各问题测试文件：`test_problem1~4.py`、`test_protocol.py`（计时、丢包、拒绝、超时、边界与 HTTP 传输检查）、`test_strategy.py`；`benchmark_problem3/4.py` 等离线基准工具仅保留作排查，不再作为常规测试流程（见上文测试策略）。
>>>>>>> b15e9cf5f3b345ae6a2e9132e036edb8871ea08f

## 文档索引

| 文档 | 内容 |
|---|---|
| [solutions/modeling-strategy.md](solutions/modeling-strategy.md) | 全局建模方案：假设、符号、四问模型主线 |
| [solutions/problem2_flow.md](solutions/problem2_flow.md) / [problem3_flow.md](solutions/problem3_flow.md) / [problem4_flow.md](solutions/problem4_flow.md) | 分问求解流程、实现与运行结果 |
| [solutions/REVIEW-2026-09-10.md](solutions/REVIEW-2026-09-10.md) 等 `REVIEW-*.md` | 分问建模审查结论与修正意见 |
| [codes/README.md](codes/README.md) | 通信层、离线桩、演练流程与代码接入的完整说明 |
| [figures/README.md](figures/README.md) | 图件总索引（数据来源、可支持结论、论文建议位置） |
<<<<<<< HEAD
| [output/README.md](output/README.md) | 输出目录约定（离线 vs 在线二分） |
=======
| [output/README.md](output/README.md) | 输出目录约定（演练记录 vs 本地结果） |
>>>>>>> b15e9cf5f3b345ae6a2e9132e036edb8871ea08f
| [tester/README.md](tester/README.md) | 模拟器与演练自动化技术说明 |

## 红线与注意事项

- **正式测试**不在本工程自动化范围内：任何代码路径都不得触碰模拟器界面的"正式测试"入口；四个 HTTP 接口无法判断演练/正式，必须由人工在界面上核实处于演练后再运行。
- `--confirm-practice` 是用户确认标志，不是程序检测到演练的证据；网络层只接受明确的回环 HTTP 地址，禁止环境代理与重定向。
- 离线桩是协议测试替身而非官方模拟器复刻，其统计量不能作为题面要求的演练成绩。
<<<<<<< HEAD
=======
- **测试一律在服务器"演练模式"进行（2026-09-12 起）**：本地只保留正确性单元测试，通过后直接上演练；不再执行本地离线基准/扫档，历史离线记录已彻底删除。
>>>>>>> b15e9cf5f3b345ae6a2e9132e036edb8871ea08f
- 出现 pending 动作或异常时，入口会停止并保留日志；恢复只能用原 client 的 `retry_pending()`，不要重建客户端"跳过"不确定动作。
- `tester/JammersSimulatorData/` 下的 sqlite 队列文件是模拟器运行数据，随演练更新，属正常现象。
