# 本地训练指南（不上服务器的演练模拟器）

`codes/local_simulator.py` 是对官方 `jammers-simulator.exe` **演练模式**的逆向复刻：
随机数算法、场景生成分布、空间示向度噪声、动作判定与虚拟时间计费、HTTP 协议均与官方一致。
不连服务器、不开官方模拟器，就能用 `codes.run_robot` + 你的策略代码跑完整演练。
算法依据与验证证据见 [reverse/NOTES.md](reverse/NOTES.md)。

> 红线不变：本工具只用于**演练性质的自测与策略迭代**。论文与正式成绩一律以服务器"演练模式"
> 实测为准；本地模拟器与"正式测试"毫无关系，也不要在任何正式流程中引用它的数字。

## 快速开始

在项目根目录开两个终端：

```powershell
# 终端 1：启动本地模拟器（问题三）
python -X utf8 -m codes.local_simulator --problem 3 --port 2027
```

```powershell
# 终端 2：与服务器演练完全相同的命令，只多一个 --base-url
python -X utf8 -m codes.run_robot --mode practice --problem 3 `
  --strategy codes.strategy_p3:solve --confirm-practice `
  --base-url http://127.0.0.1:2027 --log output/protocol/localsim-p3-01.jsonl
```

跑完会得到与官方演练同构的产物：`output/protocol/localsim-p3-01.jsonl`（协议日志）、
`output/Problem3/mission_p3_*.json`（任务记录，可用 `codes.problem4_plotting` 等现有工具处理）。

> 端口说明：默认端口 2026。**若官方模拟器正开着会占用 2026**（报 `WinError 10013`），
> 给本地模拟器换一个端口（如 2027）并让 `--base-url` 与之对应即可，两者互不影响。

## 固定场景复现（策略 A/B 对比的关键）

默认每局随机生成场景（源数 10..16、信道、位置、接收半径都不同）。要对比两个策略版本，
必须固定同一场景：

```powershell
# 方法一：固定 32 字节生成器密钥（64 位 hex），同一密钥 + 同一问题号 = 同一场景
python -X utf8 -m codes.local_simulator --problem 3 --port 2027 --key-hex 00112233aabbcc<...64位>
```

```powershell
# 方法二：直接载入已保存的场景 JSON（内含密钥，可精确重放）
python -X utf8 -m codes.local_simulator --problem 3 --port 2027 --scenario-in output/localsim/scenario-p3-xxx.json
```

每次启动都会把本次场景（**含干扰源真值与密钥**）写入 `output/localsim/`（已 gitignore，勿外传、勿入库）。
批量扫档示例：

```powershell
foreach ($k in 1..20) {
  $seed = "{0:x64}" -f $k
  Start-Process -NoNewWindow python -ArgumentList "-X","utf8","-m","codes.local_simulator",`
    "--problem","3","--port","2027","--key-hex",$seed
  python -X utf8 -m codes.run_robot --mode practice --problem 3 `
    --strategy codes.strategy_p3:solve --confirm-practice `
    --base-url http://127.0.0.1:2027 --log "output/protocol/localsim-p3-batch$k.jsonl"
}
```

（每个种子跑完再换下一个；或手动改端口并行，注意日志文件名不要覆盖。）

## 与官方模拟器的对应关系

| 官方行为 | 本地复刻 |
|---|---|
| 场景随机数（HMAC-SHA256 计数器源） | 完全一致（含域分隔符与流标签） |
| 源数 10..16、20 信道洗牌、1770 m 圆盘落点、接收半径 1000..1500 m | 完全一致 |
| 示向度噪声（BLAKE2b-64 值噪声、150 m 格距、smoothstep、±1° 限幅、0.01° 量化） | 一致（三角函数 ≤1 ulp 差异，个别示向度可能差 0.01°） |
| 虚拟时间：5 m/s 移动、measure 5 s、换信道 +1 s、clear 5 s/落空 3 s、near ≤5 m、清除半径 20 m | 一致（已用真实演练日志 248 条转移逐项验证） |
| 登录、授权票据、加密行为日志、上传队列 | 不复刻（本地演练不需要） |

其他差异：`/enter` 的 `max_real_duration_s` 固定 1200 s；不复刻 GUI、倒计时窗口与掉线重连行为。

## 自检

```powershell
python -m unittest codes.test_local_simulator -v
```

其中 `test_timing_matches_recorded_session` 会用 `output/protocol/practice-p3-*.jsonl`
的真实服务器演练日志重放计时模型，日志不存在时自动跳过。

## 文件索引

- `codes/local_simulator.py` — 模拟器本体（RNG / 场景生成 / 引擎 / HTTP），含 CLI。
- `codes/test_local_simulator.py` — 正确性测试。
- `reverse/NOTES.md` — 逆向结论：算法、常数、证据与官方二进制地址对照。
- `reverse/annotated/`、`reverse/disasm/`、`reverse/symbols/` — 关键函数标注反汇编与符号表（原始产物不入库）。
- `jammers-simulator.exe`、`README.md`、`runq3.txt`、`runq4.txt` — 官方模拟器与服务器演练流程（见 [README.md](README.md)）。
