# 问题二图件说明

`problem2_candidates.png` 与 `.svg` 是同一张图的预览版和矢量版，由 `codes/plotting.py::plot_problem2_regions` 生成，底层求解来自 `codes/problem2_solution.py`。

| 文件 | 内容与编码 | 数据来源/脚本 | 可支持结论 | 建议位置 | 使用限制 |
|---|---|---|---|---|---|
| `problem2_candidates.png` / `.svg` | 灰点＝第一次可能区域，蓝点＝可靠检测点，紫点＝满足相对质量阈值的较优候选点，蓝方块＝第一检测点，红星＝最终推荐第二检测点，虚线圆＝目标区域边界 | `codes/plotting.py`；配置与结果由 `codes/problem2_solution.py` 计算，补充结果见 `output/Supplement/supplement_results.json` | 直观展示候选筛选、可靠接收约束和最终选点之间的关系，可支持“推荐点位于可靠候选区域内”的几何解释 | 问题二候选点筛选和结果展示 | 点云是离散网格结果，不是连续可行域；图面不单独证明全局最优，最坏定位直径和约束应引用数值表 |

坐标单位为米，标题中的覆盖裕量来自运行时结果。改变最小接收半径、网格步长或候选阈值后必须重绘，不能继续沿用旧图注。
