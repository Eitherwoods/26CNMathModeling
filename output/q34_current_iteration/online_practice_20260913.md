# q3/q4 线上练习演练记录（2026-09-13）

入口：`python -m codes.gui_autopilot drill/run --problem N`，内部严格调用 `codes.run_robot --mode practice --confirm-practice`。本轮未触碰正式测试。

| 题目 | 轮次 | 案例规模 | 清除数 | 虚拟总时间(s) | 平均每源(s) | 停止原因 | 未解析 | planner_errors | 程序运行(s) |
|---:|---:|---|---:|---:|---:|---|---|---:|---:|
| q3 | 1 | 共10个， 全向10个， 定向0个 | 10 | 3885.412 | 388.541 | `all_channels_resolved` | 空 | 0 | 20.609 |
| q3 | 2 | 共14个， 全向14个， 定向0个 | 14 | 3937.960 | 281.283 | `all_channels_resolved` | 空 | 0 | 27.547 |
| q3 | 3 | 共11个， 全向11个， 定向0个 | 11 | 3509.271 | 319.025 | `all_channels_resolved` | 空 | 0 | 26.750 |
| q4 | 1 | 共12个， 全向1个， 定向11个 | 12 | 9248.447 | 770.704 | `all_channels_resolved` | 空 | 0 | 5.469 |
| q4 | 2 | 共11个， 全向3个， 定向8个 | 11 | 8456.083 | 768.735 | `all_channels_resolved` | 空 | 0 | 4.187 |
| q4 | 3 | 共16个， 全向2个， 定向14个 | 16 | 8479.202 | 529.950 | `cleared_limit` | 1,10,12,14 | 0 | 5.453 |
| q4 | 4 | 共12个， 全向1个， 定向11个 | 12 | 8687.222 | 723.935 | `all_channels_resolved` | 空 | 0 | 4.281 |
| q4 | 5 | 共15个， 全向13个， 定向2个 | 15 | 8343.335 | 556.222 | `all_channels_resolved` | 空 | 0 | 9.985 |

## 平均结果

- q3：3/3 轮 `all_channels_resolved`，平均每源 **329.616 s**；按总时间/总清除数加权为 **323.790 s/源**。
- q4：5/5 轮按策略完成，平均每源 **669.909 s**；加权为 **654.762 s/源**。其中 4/5 轮为 `all_channels_resolved`，1 轮达到 q4 允许的 16 源 `cleared_limit`，该轮未解析通道为 1、10、12、14，未计为算法异常。
- 全部线上轮次 `planner_errors=0`，`inconsistent_channels` 为空；q3 三轮无未解析通道，q4 的未解析通道只出现在上限停止轮。

## 审计文件

- 每轮任务记录：`output/Problem3/mission_p3_*.json`、`output/Problem4/mission_p4_*.json`。
- 协议日志：`output/protocol/practice-p3-01.jsonl`、`output/protocol/practice-p4-01.jsonl`。
- 固化汇总：`online_practice_20260913.json`。
- 第五轮 q4 已验证路径编码回退，成功生成 `output/Problem4/mission_p4_20260913-003235.txt`。
