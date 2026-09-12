# output/protocol — 演练协议日志

本目录存放与官方模拟器之间逐条收发的**原始协议日志**（`.jsonl`，一条一行请求/响应流），
由 `run_robot --mode practice` 的 `--log` 参数指定写入，演练（含 GUI 自动化）都写这里。

> **测试策略（2026-09-12 起）**：测试一律在服务器"演练模式"进行，本目录只保留演练协议日志。
> 历史离线自检记录（`offline-*.jsonl`、`offline-mission_*.json`、`q34_iteration/` 基准结果）
> 已于 2026-09-12 **彻底删除**；单元测试与离线 CLI 重新生成的 `offline-*`、`test-*.jsonl`
> 属本地测试产物，已加入 `.gitignore`，不再入库。

## 命名规则

| 前缀 | 含义 | 示例 |
| --- | --- | --- |
| `practice-*.jsonl` | 模拟器演练的协议日志（演练成绩的任务记录在 `output/ProblemN`） | `practice-p4-01.jsonl` |
| `offline-*`、`test-*.jsonl` | 本地测试产物（离线桩/单元测试生成，**不入库、不作成绩**） | `test-problem3-offline.jsonl` |

演练任务记录（`mission_pN_<时间戳>.json` + 界面导出 `.txt`）在 `output/Problem3|4/`，
见 `output/README.md`；协议日志的落盘路径由运行命令的 `--log` 参数决定。

## 现有文件

| 文件 | 来源 |
| --- | --- |
| `practice-p3-01.jsonl` | `tester/runq3.txt` / GUI 自动化的问题三真实演练，多次会话追加；"单动作实测 ~117 ms 墙钟"结论的原始依据 |
| `practice-p3-demo.jsonl` | `codes/README.md` 里的演练通信示例（`strategy_demo` 4 个固定动作） |
| `practice-p4-01.jsonl` | `tester/runq4.txt` / GUI 自动化的问题四真实演练，多次会话追加 |

## 常用命令

```powershell
# 演练（必须人工确认模拟器已在演练模式；不要点正式测试）
python -m codes.run_robot --mode practice --problem 4 --strategy codes.strategy_p4:solve --confirm-practice --log output/protocol/practice-p4-01.jsonl
```
