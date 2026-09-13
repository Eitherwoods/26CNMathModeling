# output/ 目录约定

**测试策略（2026-09-12 起）**：所有测试在服务器"演练模式"进行；本地只保留正确性单元测试，
不再保留离线测试记录（历史离线记录已彻底删除，`offline-*`、`test-*.jsonl` 已加入 `.gitignore`）。

- `output/Problem3/`、`output/Problem4/`：问题三/四**在线演练**的任务记录，
  HTTP 协议 JSON（`mission_pN_<时间戳>.json`）加界面导出的 TXT 动作记录。
  **根目录不堆散文件**：每批演练结束后归档到 `testN/` 子目录（N 递增）。
  归档约定见 `body/main.tex` 与 `solutions/` 中的结果引用，勿跨批次混放。
- `output/protocol/`：演练的原始协议日志（`practice-*.jsonl`），见其 [README.md](protocol/README.md)。
- `output/Problem1/`、`output/Problem2/`：问题一/二本地几何求解结果（不涉及模拟器）。
- `output/Supplement/`：补充分析结果。
- `output/localsim/`（约 372 MB）、`output/protocol/q34_iteration/`（约 54 MB）、
  `output/q34_current_iteration/`（约 7 MB）：本地模拟器场景与迭代缓存，
  均已在 `.gitignore` 内（localsim 含干扰源真值），不入库、可随时清空。

## 归档批次索引

| 目录 | 时间范围（本地 GMT+8） | 文件数 |
|---|---|---|
| `Problem3/test1` | 09-11 10:52 — 09-11 12:18 | 10 |
| `Problem3/test2` | 09-11 16:32 — 09-11 16:41 | 6 |
| `Problem3/test3` | 09-11 19:09 — 09-12 16:36 | 30 |
| `Problem3/test4` | 09-12 16:45 — 09-12 19:39 | 40 |
| `Problem3/test5` | 09-12 20:08 — 09-13 06:49 | 197 |
| `Problem3/test6` | 09-13 06:53 — 09-13 06:56 | 10 |
| `Problem3/test7` | 09-13 07:06 — 09-13 07:40 | 6 |
| `Problem3/test8` | 09-13 09:27 — 09-13 09:51 | 46 |
| `Problem3/test9` | 09-13 10:46 — 09-13 11:29 | 18 |
| `Problem4/test3` | 09-11 14:15 — 09-12 16:33 | 25 |
| `Problem4/test4` | 09-12 16:48 — 09-12 16:49 | 4 |
| `Problem4/test5` | 09-12 17:05 — 09-13 05:26 | 24 |
| `Problem4/test6` | 09-13 06:58 — 09-13 07:02 | 10 |
| `Problem4/test7` | 09-13 10:53 — 09-13 10:58 | 10 |

新增批次时自建下一个 `testN/`，并回填本表。
