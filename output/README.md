# output/ 目录约定

**测试策略（2026-09-12 起）**：所有测试在服务器"演练模式"进行；本地只保留正确性单元测试，
不再保留离线测试记录（历史离线记录已彻底删除，`offline-*`、`test-*.jsonl` 已加入 `.gitignore`）。

- `output/Problem3/`、`output/Problem4/`：问题三/四**在线演练**的任务记录，
  HTTP 协议 JSON（`mission_pN_<时间戳>.json`）加界面导出的 TXT 动作记录；
  历史演练按批次归档在 `test1/`、`test2/`、`test3/` 子目录。
- `output/protocol/`：演练的原始协议日志（`practice-*.jsonl`），见其 [README.md](protocol/README.md)。
- `output/Problem1/`、`output/Problem2/`：问题一/二本地几何求解结果（不涉及模拟器）。
- `output/Supplement/`：补充分析结果。
