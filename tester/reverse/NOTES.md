# jammers-simulator.exe 逆向结论笔记

记录 `jammers-simulator.exe`（2026 国赛 B 题官方演练模拟器，v1.1.0，build b625dc14）的
逆向分析结论。本地复刻实现见 [codes/local_simulator.py](../../codes/local_simulator.py)，
使用说明见 [../READMElocal.md](../READMElocal.md)。

## 1. 对象识别与工具链

- **格式**：PE x86-64，约 18.8 MB，Go 编译（pclntab magic `0xFFFFFFF1` 位于文件偏移
  `0x931900`，19232 个函数），**Wails v3 (beta.18)** 桌面应用（Go 后端 + WebView2 前端）。
- **模块名**：`jammers/client`。符号未混淆、未 strip（pclntab 完整），函数名明文可读。
- **关键内部包**：
  - `internal/scenario` —— 场景生成（`GeneratePractice`、`counterSource` 随机源）
  - `internal/bearingnoise` —— 示向度噪声（`grid`/`ErrorDegrees`/`QuantizeBearingHundredths`）
  - `internal/simcore` —— 仿真引擎（`Engine.Apply/measure/clear/moveTo` 等）
  - `internal/robotapi` —— 机器人 HTTP 接口；`internal/testsession` —— 会话/期限
  - `internal/runcontroller`、`internal/auth`、`internal/remoteapi` —— 演练编排与服务器交互
- **工具链**（全部产物在 `tester/reverse/`，脚本 `parse_pclntab.py` 可复现）：
  1. 解析 pclntab → `symbols/all_symbols.txt`、`symbols/entry2name.json`（函数入口 → 符号名）。
  2. `objdump -d -M intel`（mingw64）按函数范围反汇编 → `disasm/`。
  3. 脚本回填 call 目标符号、字符串常量、float 常量 → `annotated/*_ann.asm`。
  4. 全量反汇编 `full_disasm.txt`（约 123 MB，可用
     `objdump -d -M intel jammers-simulator.exe > full_disasm.txt` 再生成）。

## 2. 随机源：`scenario.(*counterSource)`

结构体 `counterSource{ key [32]byte; counters map[string]int64 }`，密钥 32 字节。

```
next(label) uint64:
    c := counters[label]                       # 每条流独立计数器，从 0 起
    counters[label] = c + 1
    msg = "practice-case-v1\x00" + label + "\x00" + be64(c)
    return be64( HMAC-SHA256(key, msg)[:8] )   # crypto/hmac.New(crypto/sha256.New, key)
```

- 域分隔符 `practice-case-v1\x00`（17 字节，含 rodata 的 NUL）位于 `0x1407fc7d2`。
- `uintN(label, n)`：无偏取模。`threshold = (2^64 - n) mod n`（= 2^64 mod n），
  `v = next(label)`，`v < threshold` 则重抽（重抽同样推进计数器），返回 `v mod n`；
  `n = 0` 或 `n > 2^63` panic。
- `shuffle(label, seq)`：Fisher–Yates，`for i := len-1; i > 0; i-- { j := uintN(label, i+1); swap }`。

**密钥来源**：`runcontroller.(*Controller).StartPractice` 中
`io.ReadAtLeast(reader, buf, 32)`，reader 为 Controller 注入的 `io.Reader`，
`runcontroller.New` 缺省填 `crypto/rand.Reader`（全局变量 `0x14117e420`，由
`crypto/rand.init` 写入）。⇒ 场景事前不可预测；但密钥以 `generator_seed_hex`（64 位 hex）
写进场景 JSON，拿到场景即完全可复现。

## 3. 场景生成：`scenario.GeneratePractice`

签名（寄存器 ABI 还原）：`GeneratePractice(problem byte, key [32]byte,
genRules PracticeGenRules(0x70), simRules SimulationRules(0x80))`。
`problem ∈ {3,4}` 否则报错；两张规则结构体与内置模板逐字节比较（防版本漂移）。

抽取顺序（`counterSource` 流标签即 `next`/`uintN` 的 key）：

1. `count = uintN("count", 7) + 10` → 源数 **N ∈ [10,16]**。
2. `channels = shuffle("channels", [1..20])`，取前 N 个后 **升序**（`sort.Ints`）分配给源。
3. （仅 problem=4）`dcount = uintN("directional-count", N)`；
   `pool = shuffle("directional-channels", 升序信道表)`；前 dcount 个信道为
   **directional**，其余 **omni**（P3 全部 omni，实测 result.json 一致）。
4. 逐源（信道号 ch）：
   - 落点（拒绝采样，成对重抽直到合法）：
     - `radius = 1770000000.0 · sqrt( (next("jammer/%d/radius") >> 11) · 2^-53 )`（um）
     - `theta  = (next("jammer/%d/theta") >> 11) · 2^-53 · 2π`
     - `x = round(cos·radius)`, `y = round(sin·radius)`（四舍五入远离零，um）
     - 接受条件 `insideJammerDisk`：`|x|² + |y|² ≤ 1770000000²`（math/big 精确整数）
   - `max_receive = 1000000000 + uintN("jammer/%d/receive", 500000001)`（um，含两端
     → **[1000, 1500] m** 的均匀整数）
   - （仅 directional）`direction = uintN("jammer/%d/direction", 360000000)`（udeg，[0,360°)）
5. `noise_seed = next("noise-seed")`，场景字段 `noise_seed_hex = "%016x"`。
6. 自检：`ParseAndValidate`（≤0xF000 字节、JSON 解码、canonical 重编码比对）后返回。

场景 JSON 还含 `schema:"scenario-v1"`、`problem`、`generator:"practice-gen-v1"`、
`generator_seed_hex`（= 32 字节密钥的 hex）、gen/sim 规则结构体与 jammers 数组
（channel、x_um、y_um、max_receive_um、kind、direction_udeg）。

## 4. 规则模板常数

**practice-gen-rules-v1**（`0x140907530`，0x70 字节）：

| 偏移 | 值 | 含义 |
|---|---|---|
| +0x00 | "practice-gen-rules-v1" | 版本串 |
| +0x10 | 1 800 000 000 um = 1800 m | 用途未定位 |
| +0x18 | 1 770 000 000 um = 1770 m | 落点圆盘半径（uniform_disk_area） |
| +0x20 | 30 000 000 um = 30 m | 用途未定位 |
| +0x28 | 335 613 962 um ≈ 335.614 m | 用途未定位 |
| +0x30 | 1 000 000 000 um = 1000 m | 接收半径下限（uniform_integer_um） |
| +0x38 | 1 500 000 000 um = 1500 m | 接收半径上限 |
| +0x40/+0x50/+0x60 | 枚举串 | "uniform_disk_area" / "uniform_integer_um" / "uniform_integer_one_to_count" |

**simulation-rules-v1**（`0x140907ea0`，0x80 字节）——Engine 把它内嵌在自身 +0x10 起：

| 偏移 | 值 | 含义（Engine 字段 = 引擎+0x10+偏移） |
|---|---|---|
| +0x00 | 1800 m | 场地尺度（推断，状态校验用） |
| +0x08 | 5 m | near 判定半径（距源 ≤5 m → `measure_result:"near"`） |
| +0x10 | 20 m | **clear 成功判定半径** |
| +0x18 | 5 um/us = 5 m/s | 移动速度 |
| +0x20 | 1 s | measure 换信道罚时 |
| +0x28 | 5 s | measure 计费 |
| +0x30 | 5 s | clear 成功计费 |
| +0x38 | 3 s | clear 落空计费 |
| +0x40 | 360 000 s | 虚拟时长上限（/enter `max_virtual_duration_s`） |
| +0x48 | 180 s | 用途未定位 |
| +0x50 | "spatial-bearing-v1" | 噪声模型标识 |
| +0x60 | 150 m | 值噪声格距 |
| +0x68 | 1 m | 用途未定位 |

## 5. 示向度噪声：`internal/bearingnoise`

- `grid(seed int64, ch byte, i, j int64) float64 ∈ [-1,1)`：
  `u = BE64( BLAKE2b(摘要 8 字节, "%d:%d:%d:%d" % (seed, ch, i, j)) ) / 2^64`，返回 `2u-1`。
  （x/crypto/blake2b `newDigest(8, nil)`；格式串 `"%d:%d:%d:%d"` 位于 `0x1407f747b`。）
- `ErrorDegrees(seed, ch, x, y)`：坐标除以 **150 m** 取格；对四格点
  `(i,j),(i+1,j),(i,j+1),(i+1,j+1)` 做 **smoothstep（3t²−2t³）双线性插值**；分数部分钳到 [0,1]。
  返回值即 ±1° 量级的角误差（度）。
- `QuantizeBearingHundredths(bearing, err)`：
  `s = round((bearing+err)·100)`（四舍五入远离零），
  钳位 `ceil((bearing−1)·100) ≤ s ≤ floor((bearing+1)·100)`（即误差限幅 ±1°），
  `mod 36000` → 单位 0.01° 的整数；HTTP 返回 `svd_deg = s/100 ∈ [0,360)`。
- `directionalCoverage`：`"omni"` 恒可见；directional 判
  `|remainder(angleToRobot − direction_deg, 360)| ≤ 90.000000001`（固定 ±90° 半平面）。
- `measure` 判定链（信道内部 0 基传给噪声、1 基做校验/位图）：
  未知信道或已清除 → `no_signal`；`d > max_receive` → `no_signal`；
  波束外 → `no_signal`；`d ≤ 5 m` → `near`（无示向度）；否则 `direction` + `svd_deg`。
  （P3 全 omni，实际只会出现 direction/no_signal。）

## 6. 引擎计费与会话（实测验证）

请求携带**绝对坐标**（m，浮点）。每次动作：

```
t += round_µs( hypot(x−px, y−py) · 2e5 )     # 移动，5 m/s，±0 归一
measure：t += 1 s（若 channel ≠ lastChannel，随后更新 lastChannel）+ 5 s
clear ：t += 5 s（成功：d ≤ 20 m 且存在且未清除，置位已清除位图）/ 3 s（落空）
        clear 无换信道罚时、不更新 lastChannel；lastChannel 初值 = 信道 1
t ≥ 360 000 s → 会话 virtual_timeout 结束
```

- 坐标合法性：有限数且 `|x|,|y| ≤ 2×10^6`（m）；信道 1..20，否则拒绝。
- 虚拟时间以 µs 为整型累计，HTTP 返回 `virtual_time_s`（整秒不带小数，否则 6 位小数，
  `robotapi.virtualSeconds` 的 `%d.%06d`）。
- HTTP 契约（与 `codes/protocol.py` 对应，样本见 `output/protocol/practice-p3-*.jsonl`）：
  - `/enter` → `{accepted, real_timestamp_ms, virtual_time_s, max_virtual_duration_s:360000,
    max_real_duration_s:1200, remaining_real_duration_s}`
  - `/measure` → `+{measure_result, svd_deg?}`；`/clear` → `+{clear_result}`；
    `/exit` → `+{exit_reason:"user_exit"}`
- **验证**：真实演练日志 `practice-p3-01.jsonl` 的 248 条相邻动作转移全部与上式
  逐微秒吻合（测试 `codes.test_local_simulator.TimingReplayTests`）。

## 7. 复刻状态与已知差异

`codes/local_simulator.py` 按上述结论实现，18 项单元测试通过；端到端演练（真实
`strategy_p3` 打本地模拟器）清除 16/16 源，清除信道与场景真值逐一相符。

已知差异：Go 与 CPython 的 `sin/cos/atan2` 存在 ≤1 ulp 差异，极端情况下个别格点噪声或
量化边界差 0.01°；登录/票据/加密行为日志/上传队列不复刻；`gen_rules` 中 3 个未定位常数
（1800 m、30 m、335.6 m）与 `sim_rules` 中 180 s、1 m 两项不影响 P3 演练路径。
