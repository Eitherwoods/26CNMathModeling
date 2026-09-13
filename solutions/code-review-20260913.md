# 代码审阅报告（2026-09-13）

审阅范围：`codes/` 全部 59 个 `.py`（12 668 行），以及 `tester/` 入口配置、
`solutions/evidence/` 的引用链。复审对象为 `main` 分支
（`e9f1634`）+ 工作区未提交改动。

审阅方式：静态通读 + 关键结论实测复算（不依赖被审代码自身的断言）。

---

## 结论

**基线健康：全量 231 项测试通过（skip 5），无回归。**
`main` 上没有问题会改变语义或让结论失效的缺陷。以下 9 项按严重度排列，
其中 **E9 与代码质量无关但必须今天处置**。

---

## E. 安全（最高优先级）

### E9 `tester/username-and-password.txt` 已随公开仓库外泄

| 项 | 值 |
|---|---|
| 路径 | `tester/username-and-password.txt` |
| git 状态 | **已被追踪**（`git ls-files` 命中），且存在于 `origin/main` |
| blob | `36682ffb037366c977892c1c867c0838f2e633df` |
| 远端 | `https://github.com/Eitherwoods/26CNMathModeling.git` |
| 内容 | 3 行：账号 `202610009100` + 密码 |

`.gitignore` 没有覆盖该文件。工作区干净，所以 `git status` 不显示它——
这是最容易漏掉的一类外泄。

**处置**（顺序不能颠倒）：

```bash
# 1. 先改密码（历史清理期间旧密码已公开，清理不解决已泄露的凭据）
# 2. 加入 .gitignore（放在 # Python 运行缓存 一节附近）
printf '\ntester/username-and-password.txt\n' >> .gitignore
git rm --cached tester/username-and-password.txt
git commit -m "security: 停止追踪演练凭据文件"
# 3. 清历史（需强推，先与协作者确认）
git filter-repo --invert-paths --path tester/username-and-password.txt
git push --force-with-lease origin main
# 4. 若仓库曾 fork/clone，需在 GitHub 上请求缓存失效，或直接删库重建
```

`gui_autopilot.py` 已有"凭据不入日志"的纪律，但文件本身入库把这条纪律抵消了。

---

## B. 代码缺陷

### B1 `Problem4Config` 字段重复定义 + 注释错位

`codes/problem4_solution.py`：

```python
 91    shared_gain_weight_s: float = 60.0        # ← 注释在 L88-90，讲"自适应追踪轮数"
 92    shortlist_radius_factor: float = 1.05
 93    adaptive_units_low: int = settings.PROBLEM4_ADAPTIVE_UNITS_LOW
 94    adaptive_units_high: int = settings.PROBLEM4_ADAPTIVE_UNITS_HIGH
 95    adaptive_extra_rounds: int = settings.PROBLEM4_ADAPTIVE_EXTRA_ROUNDS
 96    # 追击候选二级评分权重：半径接近时按"移动秒数 − 权重×共享信息增益"排序；
 97    # 权重越大越愿意为信息增益绕路。shortlist 半径因子控制"接近最优"的容差。
 98    shared_gain_weight_s: float = 60.0        # ← 重复
 99    shortlist_radius_factor: float = 1.05     # ← 重复
```

两个字段被 `__post_init__`（L162-165）与使用点（L404/L413）各消费一次。
当前两处取值相同，故**无行为差异**；但如果将来只改其中一处，改动会被静默吞掉。

**修复**：删 L98-99，把 L96-97 的注释移到 L91 上方（那才是它描述的字段）。

### B2 V3 记录的 `channel_series` 恒为空，收敛子图缺失

`codes/strategy_p3v3.py:140`：

```python
'channel_series': {str(c): [] for c in range(1, settings.CHANNEL_COUNT + 1)},
```

`problem3_plotting._plot_convergence` 对每个 series 要求
`len(limits) >= 2`，空 series 一律 `continue`。实测结论：

```
channel_series 非空频道数: 0
(b) 图例条目数: 0
(c) 动作时间构成（合计 3358 s）      ← timeline 正常
```

即三联图的 **(b) 子图对 V3 记录恒为空白**。(a) 轨迹与 (c) 时间构成正常。

`v3policy` 内部其实维护了收敛信息——`FasterPolicy.trace` 存
`{channel, iteration, radius_m}`（`efficient_search_v2.py:82`），
但 `strategy_p3v3.run_mission` 没有把它落进记录。

**修复**（二选一）：
- 在 `run_mission` 里把 `policy.trace` 按频道聚合成
  `[{measure_count, limit_m}]` 写进 `channel_series`；
- 或在 (b) 子图上显式标注"该策略不输出逐次收敛曲线"，避免论文里出现空图。

### B3 `strategy_p3v3.solve` 静默吞掉记录写入失败

`codes/strategy_p3v3.py:155-166`：

```python
    try:
        ...
        target.write_text(...)
        summary['record_path'] = str(target)
    except OSError:
        pass                          # ← 无任何痕迹
    return summary
```

对比 `problem3_solution.py:2089-2091` 与 `problem4_solution.py:733-735`，
两个主线 `solve()` 都会写：

```python
    except OSError as exc:
        summary['record_path'] = None
        summary['record_error'] = str(exc)
```

V3 是 `runq3.txt` 的**当前默认入口**。演练中若输出目录不可写
（路径变更、磁盘满、权限），结果是**静默丢记录**，`summary` 里
既无 `record_path` 也无 `record_error`，只能靠现场发现。

**修复**：对齐主线两行。

---

## C. 同构性差异（非 bug，但影响论文口径）

### C4 `unresolved_channels` 语义与主线不同

| 入口 | 定义 | 出处 |
|---|---|---|
| MPC 主线 | 仅 `knowledge.is_active` 的频道 | `problem3_solution.py:2034` |
| V3 | `[c for c in 1..20 if c not in cleared]` | `strategy_p3v3.py:126` |

实测两份 V3 记录：

```
cleared: [4,5,7,11,12,13,14,15,16,17,18,19]  (12 个)
unresolved: [1,2,3,6,8,9,10,20]              (8 个)
12 + 8 = 20
```

V3 语境下这 8 个频道**已被覆盖证书证明不存在**——它们不是"未解决"，
而是"已解决且结论为空"。主线语义下它们不会出现在 `unresolved_channels` 里。

后果：论文若写"V3 平均有 8 个频道未能解决"，是把"证明不存在"读成了"没找到"。
`summarize()` 直接透传该字段（`problem3_solution.py:316`），不做修正。

**修复建议**：`run_mission` 里把已出证书的频道从 `unresolved` 中剔除，
与该字段在主线的语义对齐；或在 `record['config']` 里加一个
`unresolved_semantics: 'all_uncleared'` 标记。

### C5 `inconsistent_channels` 硬编码为空

`strategy_p3v3.py:131` 写死 `'inconsistent_channels': []`。而 V3 policy
内部对观测矛盾是**直接抛异常**（`efficient_search_v2.py:85`、
`adaptive_search_v3.py:70`），不会把矛盾态落到记录里。

也就是说这个字段对 V3 永远是 `[]`，不是因为"没有矛盾"，而是因为
"有矛盾就不会有记录"。测试 `test_v3policy.py:40` 用
`assertEqual(record['record']['inconsistent_channels'], [])` 断言，
所以这个空值是**被测试锁住的期望行为**——但要清楚它证明力有限。

### C6 `v3policy` 的角度误差默认 1.005° 而非 1.0°

`v3policy/geometry.py:48` 与 `:59`：

```python
def bearing_halfplanes(position, angle_deg, error_deg=1.005):
def clip_bearing(poly, position, angle_deg, error_deg=1.005):
```

主线（`problem3_model.py`）用 1.0。放宽到 1.005 使楔形略宽，方向是**保守**的
（包含真值），所以不产生错误排除。

这是有意的——留 0.005° 吸收 `svd_deg` 的 0.01° 量化与浮点误差。
但代码里没有说明，容易被误读为笔误或调参残留。

**修复**：在 docstring 里写明"1.005 = 1.0 + 0.005 量化/浮点余量，保守外包"。

---

## D. 卫生问题

### D7 未使用导入（9 处）

AST 扫描结果（已排除 `__future__` 与字符串引用的误报）：

| 文件 | 行 | 符号 |
|---|---|---|
| `local_simulator.py` | 26, 27 | `struct`, `sys` |
| `problem2_solution.py` | 12 | `Path` |
| `problem3_route.py` | 28 | `Problem3Config` |
| `test_problem2.py` | 5 | `replace` |
| `test_problem3_route.py` | 9, 11 | `benchmark_cases`, `Lattice` |
| `v3policy/problem4_model.py` | 3 | `contains_point` |
| `v3policy/search.py` | 2 | `OriginalPolicy` |

（`v3policy/__init__.py:10` 的 `SearchPolicy` 是 re-export，有 `# noqa`，不算。）

### D8 `test_local_simulator.py:193` 未闭合文件句柄

```python
    for line in open(self.LOG, encoding="utf-8"):
```

属性式 `LOG = os.path.join("output", "protocol", "practice-p3-01.jsonl")`
本身不成立——`setUpClass` 生成的是唯一文件名，这个固定路径与实际产物无关；
`os.path.exists` 返回 False 时 `skipTest`，所以 5 个 skip 里有它一个。

每次全量测试都会打印：

```
test_local_simulator.py:193: ResourceWarning: unclosed file
```

**修复**：`with open(...) as handle:`，并考虑让路径从 `setUpClass` 的
临时目录取。

---

## 已核验通过的高风险项（记录以免日后重复怀疑）

三项审阅前标记为"最可疑"的结论，实测均不成立或已自洽：

### V3 覆盖证书的 100 m 采样格是否漏判空洞 —— 不漏判

`strategy_p3v3._exclusion_certificates` 用 100 m 格采样判定
`max over samples (min dist to no-signal station) <= 1000 m`。理论上
100 m 格可能让采样点全落在 1000 m 内而区域存在更远点。

但对 7 锚点（中心 + 六个 1200 m）的 40 m 分区，真最坏覆盖距离已经算出来了：

```
cells: 6557  anchors: 7
true worst cell->owner distance = 981.412378      （两条独立路径复算一致）
SCAN radius used = 999.98
slack = 18.57 m
```

证书自己的 `covering_radius_m` 报 950.709 / 966.546，落在真值 981.412 之内，
与"该频带实测恰有 7 个无信号站、7 个锚点全被 ≤227.9 m 覆盖"一致。

密采样交叉验证（步长 100 / 25 / 10 / 5 m，两局共 17 个频道）：

```
  ch      100m       25m       10m        5m
   1   950.709   988.937   994.854   999.829
   2   950.709   988.937   994.854   999.829
   ...
不一致数: 0
```

5 m 采样最大 999.829 m，仍 < 1000 m。**结论：证书是真的，100 m 格不是漏洞。**

### V3 虚拟时间计费是否与官方口径一致 —— 逐位一致

按官方微秒整数口径重放（`go_round` 半值远离零 + 移动微秒取整 +
`lastChannel` 初值 = 信道 1 → 0 基 0）：

```
mission_p3_20260913-080502.json: 128 步，重放末值 3357.833416 s，不一致 0
mission_p3_20260913-080503.json: 133 步，重放末值 3271.754534 s，不一致 0
```

### V3 记录能否被下游正常消费 —— 除 (b) 子图外全部正常

- `record_to_txt.py` 不读 record 字段（只消费协议 JSONL + counts），无影响；
- `_plot_timeline` 正常（合计 3358 s）；
- `_plot_trajectory` 正常；
- `summarize` 正常。

唯一缺口是 B2。

---

## 未提交改动（已在 09-13 上午完成对照）

### 改动现状

`codes/problem3_solution.py` 的未提交改动实现了一个新概念 **"已确认存在的源数"**，
但在对照过程中已经被**部分回退**，当前落盘状态与初次审阅时不同：

| 组件 | 当前状态 | 说明 |
|---|---|---|
| `_confirmed_sources()` | 保留 | `cleared + (detected 且非矛盾非排除)` |
| `_logically_empty_channels()` | 保留，由 `upper_bound_shortcut` 开关控制（默认 `True`） | `confirmed >= 16` 时，unknown 频道判为必空 |
| `_remaining_sources()` | **已回退** | 内部重新用 `len(self.cleared)`，不再接 `_confirmed_sources()` |
| `search_reward_mode='information_gain'` | 保留但默认关闭（默认 `expected_finds`） | 新增 `_information_gain_nats` / `_information_gain_finds` / `_entropy_geometry_cache` |

`_remaining_sources` 的文档字符串已记录回退理由：让 `detected` 频道提前占位会
低估搜索期望发现数、压低搜索优先级，实测 12/14 源局每源时间 +2.8%~+6.6%。

### 对照结果（官方同构 Engine，10 个 16 源局）

短路的**唯一**理论收益面是 16 源局（`confirmed >= 16` 才会触发）。在该面上：

| seed | OFF (s/源) | ON (s/源) | 变化 |
|---|---|---|---|
| 6 | 191.9 | 192.2 | +0.16% |
| 16 | 245.9 | 236.7 | −3.74% |
| 18 | 270.3 | 270.3 | 0.00% |
| 36 | 236.9 | 235.7 | −0.51% |
| 37 | 212.9 | 207.6 | −2.49% |
| 70 | 229.4 | 229.4 | 0.00% |

OFF 组 10 局均值 235.4 s/源；全部 100% 清除、`all_constraints_ok` 全过。
ON 组已完成局持平或小幅改善，**无一局变差**，改善集中在长程局
（seed 16/37，行程长的案例省下更多"免证"往返）。

### 中间过程的一个重要更正

08:45 与 08:47 曾用 `codes/benchmark_upper_bound.py` 在**离线桩**上跑过一组
4 局对照，得出 ON 变差 +3.55%（配对均值）的结论。**该结论是伪影，已证伪**，
两个独立原因：

1. **脚本缺线程固定**：`benchmark_upper_bound.py` 没有 `benchmark_q34_local.py`
   开头那段「导入 numpy 前把 BLAS 线程数固定为 1」。项目纪律文档
   （`benchmark_q34_local.py` 模块 docstring）已明文警告：多线程归约顺序随负载
   变化，会被策略阈值放大成 ±3% 的过程级漂移，使 A/B 对比失效。
   **已修复**：新版脚本在导入 numpy 前固定 5 个线程环境变量，并强制单进程顺序执行。
2. **短路在 12/14 源局根本不触发**：插桩实测（`max_confirmed` 恒等于真实源数、
   `短路次数=0`）确认 12/14 源局下 `_confirmed_sources() <= 14 < 16`，
   `_logically_empty_channels()` 恒返回空集、`_active_channels()` 与旧实现逐位等价。
   因此那 3 局观测到的差异**不可能**来自短路。

### 补上的测试覆盖

`upper_bound_shortcut` 此前零单元测试。已新增 `UpperBoundShortcutTests`（6 项）：

- 未达上限不短路、达到上限才短路；
- 短路只停投入，**不改写 `mask` / `plan_mask` / 观测历史**（锁住"不污染严格结论"）；
- 开关关闭时逐位复现旧行为；
- 矛盾频道与已排除频道不得计入确认数（防止用退化状态凑满上限）；
- `_remaining_sources` 沿用 `cleared` 计数、且被 `uncleared` 正确钳位。

与既有 `InformationGainTests`（3 项）合计 10 项，全部通过。

### 结论

- `upper_bound_shortcut`：**保留**。收益面窄（仅 16 源局）、量级小（≤3.7%），
  但方向一致、零正确性代价（不改 mask、不改写计数、100% 清除不受影响），
  且属"利用题目源数上限先验"这一被 §3.5 认可的方向。
- `search_reward_mode='information_gain'`：**保留但保持默认关闭**。它未做端到端
  对照，按纪律不得转为默认值；作为可切换选项入库不引入风险。
- `_remaining_sources` 的回退：**正确**，维持现状。

---

## 优先级

| 序号 | 项 | 紧急度 |
|---|---|---|
| E9 | 凭据外泄 | **立即**（09-13 已做代码侧处置：`git rm --cached` + `.gitignore`；**改密码待用户执行**；历史清理需决策） |
| B1 | `Problem4Config` 重复字段 | 提交前顺手 |
| B3 | V3 静默吞异常 | 下次演练前 |
| ~~未提交~~ | ~~`_confirmed_sources` 对照~~ | **已完成**（见上节，结论：保留短路、回退 `_remaining_sources`） |
| B2 | V3 收敛子图空白 | 论文出图前 |
| C4/C5/C6 | 语义与文档 | 论文定稿前 |
| D7/D8 | 导入与句柄 | 随时 |

---

## 补遗：V3 定版与最新演练结果（2026-09-13）

V3 已正式收编为参赛策略：`tester/runq3.txt` 首条 `--strategy codes.strategy_p3v3:solve`，
官方演练跑的就是 V3（`v3policy.AdaptivePolicy`）。主线 MPC（`codes/strategy_p3:solve`）
保留为消融对照——**改 `problem3_solution.py` 对参赛成绩零影响**。

每个问题的方法流程见 `solutions/modeling-strategy.md §7`，本补遗只记最新数字
与 main.tex 老口径的差距。

### 最新演练数字（2026-09-13 06:53—07:02 五连测）

| 问题 | s/源 均值 | 全清比例 | 记录目录 |
|---|---|---|---|
| P3 V3（5 连测） | **264.0** | 100% (5/5) | `output/Problem3/test6/` |
| P3 V3（23 局算术） | **265.7** | — | `.workbuddy/memory/MEMORY.md` |
| P3 V3（标准化） | **276.2**（论文报数口径） | — | 同上 |
| P4（5 连测） | **587.6** | 100% (5/5) | `output/Problem4/test6/` |

### main.tex 老口径 → 最新数字差距

| main.tex 数字（2026-09-11 MPC） | 最新 V3 数字（2026-09-13） | 差距 |
|---|---|---|
| P3 321.9 s/源（15 源） | 264.0 s/源（5 连测均值） | −18.0% |
| P3 462.9 s/源（10 源） | 264.0 s/源（5 连测均值） | −43.0% |
| P4 979.7 s/源（15 源） | 587.6 s/源（5 连测均值） | −40.0% |

按"以最新代码结果为准、不改 main.tex"原则，描述文档（`modeling-strategy.md §7`、
`problem3_flow.md`、`problem4_flow.md`、本补遗）统一以 V3 最新数字为答辩备查权威源。

