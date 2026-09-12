# B题通信层与离线桩

依据用户指定的 `CUMCM2026Problems/B题/附件/附件1.docx`、`附件2.docx`。
实现问题一、二几何求解，以及 HTTP 协议封装、离线响应与状态机验证，并提供后续问题3/4算法接入入口。
Python 3.9+，无第三方依赖。

## 测试策略（2026-09-12 起，用户指定）

- **性能与参数评价一律以服务器"演练模式"实测为准**（`--mode practice` 或 `gui_autopilot` 全自动）；
  不在本地做离线仿真、基准或扫档。
- 本地只保留**正确性单元测试**两套：`python -m unittest codes.test_problem3 -v`、
  `python -m unittest codes.test_problem4 -v`（协议层另保留 `codes.test_protocol`）。
  测试通过即直接跑线上演练。
- 离线桩（`offline_stub.py`）降级为协议测试替身，仅服务单元测试与接入冒烟；
  其 CLI 仿真与 `benchmark_*` 工具仅作故障排查。历史离线测试记录已于 2026-09-12 彻底删除，
  `offline-*`、`test-*.jsonl` 等本地测试产物已加入 `.gitignore`，不再入库。

## 运行

在项目根目录 `D:\数模\AAA26国赛\26CNMathModeling` 执行，不能先进入 `codes` 目录：

```powershell
python -m codes.run_robot
python -m unittest codes.test_protocol -v
python -m codes.problem2_solution --station 0 0 --bearing 0
python -m unittest codes.test_problem2 -v
```

演示使用内存桩，无网络连接；默认将请求和响应追加至
`output/protocol/offline-*.jsonl`（本地测试产物，已加入 `.gitignore`，不再入库）。
每局动作ID使用UUID，不读取账号密码。
验证套件另有一项真实HTTP传输检查：自行创建 `127.0.0.1:0` 临时服务，取得系统
分配的端口后连接该桩，完成后关闭；不会连接2026端口或启动官方模拟器。

## 文件职责

- `problem2_solution.py`：问题二的保守离散化候选区域与第二检测点求解器。
- `test_problem2.py`：问题二角度、近距离、可靠性裕量与端到端验证。
- `problem3_model.py`：问题三可能位置集合的栅格表示、保守更新算子、单频道判据与覆盖布站。
- `problem3_solution.py`：问题三五项决策策略、统计汇总与离线命令行入口。
- `strategy_p3.py`：问题三运行器入口适配（导出 `solve`）。
- `scenario.py`：离线演练案例生成（随机/固定），不是官方案例分布。
- `problem3_plotting.py`：问题三任务图（轨迹、可能区域收敛、动作时间构成）。
- `test_problem3.py`：问题三栅格、覆盖式不存在性证明、清除判据与离线端到端测试。
- `protocol.py`：`RobotClient`、`RobotState`、显式开启的 `HttpTransport`。
- `offline_stub.py`：串行、幂等缓存、计时、人工配置干扰源的确定性响应。
- `run_robot.py`：默认离线；支持用户手动启动的演练，无正式模式选项。
- `gui_autopilot.py`：演练 GUI 自动化（代启模拟器→登录→点演练→等接口就绪→跑 practice 命令→自动生成 TXT），2026-09-11 用户解除演练自动化禁令后新建；正式测试入口硬编码禁触。
- `record_to_txt.py`：协议 JSONL + 任务记录 JSON → 逐动作行为记录 TXT（与 `output/Problem4/mission_p4_*.txt` 同格式）。
- `credentials.py`：只解析本地凭据文件的参赛队号；不保存、传递或记录密码。
- `strategy.py`：算法加载、上下文、会话开始与正常结束。
- `strategy_demo.py`：附件的通信计时示例，不是问题3/4求解算法。
- `test_protocol.py`：计时、丢包、拒绝、超时、输入边界和HTTP传输检查。

## 问题二运行

问题二不依赖模拟器。以第一次检测点 $(x_1,y_1)$ 和示向度 $\theta_1$ 为输入：

```powershell
python -m codes.problem2_solution --station 0 0 --bearing 0
```

结果写入 `output/Problem2/problem2_results.json`，候选区域图写入
`figures/Problem2/problem2_candidates.png` 与 `.svg`。程序按离散覆盖裕量收紧可靠条件；
更改网格步长后应重新运行，并同时检查输出中的 `coverage_margin_m`。

## 代码接入

```python
from codes.protocol import RobotClient
from codes.offline_stub import OfflineStub, Source

client = RobotClient(
    "offline-team",
    OfflineStub(sources=[Source(channel=1, x=100, y=0)]),
    log_path="output/protocol/my-offline-run.jsonl",
)
client.enter()
response = client.measure(0, 0, 1)
if response["measure_result"] == "direction":
    print(response["svd_deg"])
client.exit()
```

四个调用为 `enter()`、`measure(x,y,channel)`、`clear(x,y,channel)`、`exit()`。
未提供transport时禁止发送。`RobotClient`以锁串行化整个动作和重试过程。
每次新动作生成ID；重试使用原路径、原字节及原ID。

`ActionRejected`提供 `status` 和 `response`，不自动重试HTTP错误。
409、429、5xx保留pending，避免在无法确认执行结果时发送下一个动作。
`UncertainAction`表示断线重试耗尽或响应不合法；只可用 `retry_pending()`恢复原动作。
不要重新调用 `measure()` 或重建客户端来“跳过”不确定动作。
实例内恢复不等于进程崩溃恢复：本版本不自动从日志恢复会话。

只有HTTP 200且accepted=true才更新位置、频道、已清除集合和虚拟时间。
`clear`始终不切换测向频道；失败清除仍然会移动和耗时。
`remaining_real_duration_s`属性使用单调时钟计算；进入的第一次请求开始时间作为
保守计时基点，响应重放不会延长时限。到期后禁止继续请求，包括事后exit。
虚拟检测耗时不会调用现实sleep，短暂退避仅用于网络异常重试。

## 打开软件后，如何进行HTTP演练

不需要在软件里“生成HTTP协议”。附件2已经规定好协议：模拟器是HTTP服务端，
机器狗Python程序是客户端，自动生成JSON并发送四种POST请求。
本机软件启动并不表示接口已开放，必须先启动一局演练并等到接口就绪。

1. 打开 `tester/jammers-simulator.exe`，保持联网并在界面完成登录。
2. 软件空闲时查看机器狗接口端口，默认2026；若被占用，在设置里更换并记下端口。
3. 确认 `tester/username-and-password.txt` 已由你们保存正确内容。运行器只解析“参赛队号”
   标签后的值；密码不会进入HTTP请求、日志或命令行。若实际文件在其他位置，使用
   `--credentials "路径"` 指定，不需要也不能在命令行填写队号。
4. 在模拟器选择“问题3演练测试”（验证问题4时选“问题4演练测试”）。
   **不要选择任何“正式测试”入口。** 等待数据准备和5秒倒计时，直到显示机器狗接口就绪。
5. 核对当前确实是演练后，运行命令。程序发送 `/enter`，调用指定算法，再在正常返回后发送 `/exit`。
   问题四的现成命令见 `tester/runq4.txt`，准备步骤与结束后要核对的三件事见
   `solutions/problem4_flow.md` 的"官方模拟器演练"一节。
6. 在软件“指令与反馈”中核对请求；结束后查看演练结果，同时保留本地JSONL日志。
   每次演练结束后接口关闭；下一次需在软件中重新启动演练，再运行一次命令。

下面是供用户手动运行的**演练通信示例**，不是正式测试命令：

```powershell
python -m codes.run_robot --mode practice --problem 3 --base-url http://127.0.0.1:2026 --confirm-practice --strategy codes.strategy_demo:solve --log output/protocol/practice-p3-demo.jsonl
```

终端中直接复制上面这一行：下划线是普通 `_`，地址是普通
`http://127.0.0.1:2026`，不能写成 Markdown 链接 `[http://...](http://...)`。
如果当前提示符已经是 `...\26CNMathModeling\codes>`，先执行 `cd ..` 回到项目根目录。

示例只执行附件的4个固定动作，不搜索干扰源，也不保证清除目标。
实际演练存在真实案例，示例中的频道3在清除点附近若恰有目标，耗时会不同于离线示例。
`--problem 4`只把问题编号传给算法；它不会替你在软件中切换问题或测试模式。
`--confirm-practice`是用户确认标志，不是程序检测到演练的证据。

常见连接问题：

- 连接被拒绝：检查软件是否已进入“接口就绪”、端口是否一致、是否在同一台电脑运行。
- HTTP 200但accepted=false：检查参赛队号、当前会话状态；不要将返回的虚拟时间0视为重置。
- 超时/中止后连接关闭：查看软件界面，本局不能恢复；不要通过exit查询结束原因。
- 程序异常或存在pending动作：入口会停止并保留日志，不自动发送新动作或exit。
  用户可在演练界面手动中止；代码内恢复请使用原client的retry_pending。
- 算法模块找不到：确保从项目根目录运行，模块路径为 `codes.文件名:solve`，不带 `.py`。

## 后续求解算法如何接入

运行链为 `run_robot → /enter → solve(context) → /exit`。
协议层只负责通信，运行器负责调用算法；问题1/2的几何函数可由问题3/4算法调用，无需HTTP。
后续求解算法各自新建 `codes/strategy_p3.py`（已实现）、`codes/strategy_p4.py`（待实现），
统一导出 `solve(context)`。下面只是接口用法示例，不是完整策略：

```python
def solve(context):
    # 以下只是接口用法，不是完整策略。
    observations = []
    for channel in range(1, 21):
        if context.should_stop():
            break
        response = context.measure(0, 0, channel)
        if response["measure_result"] == "direction":
            observations.append((channel, response["svd_deg"]))
        elif response["measure_result"] == "near":
            context.clear(0, 0, channel)
        # no_signal不表示该频道不存在干扰源。
    return {"observations": observations}  # 返回可JSON序列化的汇总
```

| 接口 | 含义 |
|---|---|
| `context.problem` | 本次指定的问题编号3或4 |
| `context.measure(x,y,channel)` | 移动并检测，返回附件2定义的响应dict |
| `context.clear(x,y,channel)` | 移动并尝试清除，返回响应dict |
| `context.state` | 状态副本：position、channel、virtual_time_s、cleared_channels等 |
| `context.remaining_real_duration_s` | 保守计算的剩余现实秒数 |
| `context.should_stop()` | 剩余现实时间是否进入退出预留区（默认10秒） |

算法不要调用enter/exit，不要改变client的状态，不要自己生成request_id，也不要绕过
context另发HTTP。measure/clear进入时间预留区会抛BudgetReached，由运行器接住并尝试正常退出。
算法应定期检查should_stop；无法强制打断算法内部的长时间计算，需自行分块或设置计算预算。
其他算法/协议异常直接传播，入口不擅自继续动作。正常return不等于已经清除所有目标，
任务完成判据由具体算法负责。预留时间也不能保证在网络故障时成功退出。

实现文件后，先在离线桩上做一次冒烟（这属于允许的本地正确性验证）：

```powershell
python -m codes.run_robot --mode offline --problem 3 --strategy codes.strategy_p3:solve
```

确认冒烟通过后，把上面的演练命令中的 `--strategy codes.strategy_demo:solve`
替换成 `--strategy codes.strategy_p3:solve`；问题4同理使用对应文件和编号。
问题三的 `codes/strategy_p3.py` 已实现（转发到 `problem3_solution`）；
问题四入口为 `codes.strategy_p4:solve`（转发到 `problem4_solution`），已实现并多局演练验证。
`--strategy`会导入并执行本地Python代码，只填写自己信任的模块。

## 问题三本地自检

本地只跑正确性单元测试（性能评价一律上服务器演练模式）。
模型与流程见 `solutions/problem3_flow.md`，
建模审核结论与修正意见见 `solutions/REVIEW-Q3-2026-09-11.md`。

```powershell
python -m unittest codes.test_problem3 -v
```

`problem3_solution.py` 的离线 CLI（`--seed/--sources` 等）使用本地桩 `OfflineStub`
与 `codes/scenario.py` 生成的案例，仅作故障排查用；其统计量**不是官方演练成绩**，
记录不再保留。按 `tester/README.md` 的约定，演练测试与正式测试一律由人工在模拟器界面触发。

## 问题四代码与本地自检

建模审查见 `solutions/REVIEW-Q4-2026-09-11.md`，实现、近似边界与运行结果见
`solutions/problem4_flow.md`。文件职责：

| 文件 | 用途 |
| --- | --- |
| `problem4_model.py` | 联合位置单元×类型×接收半径×朝向的保守外包、三角搜索网格。 |
| `problem4_solution.py` | 五项决策策略、离线自检入口与记录汇总入口。 |
| `problem4_plotting.py` | 任务四联图（轨迹与朝向、覆盖证据、可能位置收敛、时间构成）。 |
| `scenario_p4.py` | 混合朝向与边界朝外案例工厂；真值仅供测试端使用。 |
| `test_problem4.py` | 49 项单元、入口、绘图、记录落盘与离线端到端测试。 |

问题四本地同样只跑正确性单元测试（默认案例是一半定向、一半全向的12源混合案例）：

```powershell
python -m unittest codes.test_problem4 -v    # 默认跳过 5 项端到端；$env:RUN_PROBLEM4_E2E='1' 显式开启
```

`problem4_solution.py` 的离线 CLI（`--sources/--seed/--directional` 等）仅作故障排查；
其统计量**不是官方演练成绩**，记录不再保留。

**记录落盘**：在线演练/正式测试的记录进 `output/Problem4/`（`mission_p4_<时间戳>.json`），
任务图一律进 `figures/Problem4/`；离线 CLI 的记录带 `offline-` 前缀写入 `output/protocol/`
但已加入 `.gitignore`，不再保留。落盘目录由 `config.record_dir_for()` 决定，
`problem4_solution.solve()` 按"是否跑在 `OfflineStub` 上"自动选择，不需要人工搬。
演练记录也可单独汇总真实总数（演练结束后才能知道）：

```powershell
python -m codes.problem4_solution "output/Problem4/mission_p4_时间戳.json" --true-total 12
```

**动作数远高于问题三，但不构成真实时间风险**：排除证据要求每个频道在全部 37 个搜索
顶点各做一次无信号检测，因此案例动作数约 640～740（问题三约 250～300）。按上局问题三
真实演练日志实测的单动作墙钟（1343 动作 / 157.4 s ≈ **117 ms/动作**，其中主要是本地
规划计算），问题四一局约 **75～90 s**，20 分钟真实窗与虚拟预算都远未触及；策略在预算
不足时仍会安全收尾并标记 `budget_limit`，不会谎报完成。

演练时真正要盯的是：排除证据必须走到半径最大约 2700 m 的网格顶点（1800 m 圆域之外，
为了让边界朝外的定向源也能被正面看到）。**这一点已被 14:15 首局真实演练确认接受**——最远
2700 m 的 187 个区域外动作全部 accepted，方案不必再改。演练命令见 `tester/runq4.txt` 与
`solutions/problem4_flow.md`；图怎么读见 `figures/Problem4/README.md`。演练后可从记录直接重画
（在线记录在 `output/Problem4/`）：

```powershell
python -m codes.problem4_plotting "output/Problem4/mission_p4_时间戳.json" --name practice_p4_L2
```

## 连接约束

官方地址默认为 `http://127.0.0.1:2026`，以模拟器设置的实际端口为准。
机器狗不负责登录；登录和演练启动由用户在模拟器中操作。
`robot_id`由 `tester/username-and-password.txt` 的“参赛队号”字段自动读取，`arena_id`固定default，无密码字段。
HTTP传输需显式构造 `HttpTransport(base_url, allow_network=True)` 并传入客户端；
命令行practice模式在用户提供确认参数后构造该传输层；导入模块不会自动联网。

**四个接口无法判断正在进行演练还是正式测试。** `allow_network=True`只开启网络，
不是正式模式保护或演练模式证明。接入官方模拟器前须由用户核实界面处于演练，
任何正式测试均不在本次工作范围。禁止因为连接失败自动启动或切换测试模块。
网络层禁用环境代理与重定向，只接受明确的回环HTTP地址和端口。

## 离线桩边界

这是协议测试替身，不是官方模拟器复刻，不能拿其结果作为题面要求的演练成绩。
人工场景数量可少于10个；默认无干扰源，以复现附件计时示例。
支持全向/半平面定向覆盖、5米near、20米清除、固定可配置示向误差。
不复刻官方空间误差场、案例生成分布、认证、加密日志、窗口关闭和资源保护。
不实施所有HTTP头、嵌套深度和流量规则；这部分应以附件为准。
官方会话到期和exit后可能直接关闭接口；桩保留缓存以便单独检查重放逻辑。
官方演练已多局联调通过；桩的结论仍仅覆盖通信与状态逻辑，不能外推成绩。
