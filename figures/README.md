# 图件总索引

本目录保存论文插图及题目附件。PNG 用于预览，若同名 SVG 存在则 SVG 为可缩放的论文插入版本；同名 PNG/SVG 视为同一张图。

| 子目录 | 图件 | 类型与用途 |
|---|---|---|
| `Problem1/` | `five_stations`、`triangle` | 问题一人工验证场景与定位区域；由 `codes/problem1_solution.py` 生成 |
| `Problem2/` | `problem2_candidates` | 问题二第一次可能区域、可靠检测点与推荐第二点；由 `codes/plotting.py` 生成 |
| `Supplement/` | `heatmap_geometry`、`curve_station_count`、`sensitivity_receive_radius` | 前两问的几何规律、检测点数收敛和接收半径敏感性；由 `codes/analysis_supplement.py` 生成 |
| `Problem3/` | `sprint_tradeoff`、`practice_p3_final1/2`、`practice_p3_opt1/2/3` | 问题三速通前沿权衡折线图与在线演练配图；折线图由 `codes/sprint_frontier.py` + `codes/plot_sprint_tradeoff.py` 生成，逐图说明见该目录 README |
| `Problem4/` | `practice_p4_L1`、`practice_p4_L2`、`mission_all_channels_resolved`、`mission_cleared_limit` | 问题四在线演练或离线桩案例四联图；由 `codes/problem4_plotting.py` 及 `codes/problem4_solution.py` 生成 |
| `QuestionFile/` | `fig1.png`、`fig2.png` | 题目附件原图，非模型生成，不用于证明算法结果；逐图说明见该目录 README |

各目录中的 README 给出逐图的内容编码、数据来源、可支持结论、论文建议位置和使用限制。图中数值应以对应的 `output/` 记录和脚本参数为准，不从图片像素反推数值。
