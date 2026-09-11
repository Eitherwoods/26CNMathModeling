# 补充分析图件说明

三张图均由 `codes/analysis_supplement.py` 生成，原始计算记录在 `output/Supplement/supplement_results.json`，汇总表在 `output/Supplement/supplement_tables.md`。每个 PNG/SVG 对应同名一图。

| 文件 | 内容与编码 | 数据来源/脚本 | 可支持结论 | 建议位置 | 使用限制 |
|---|---|---|---|---|---|
| `heatmap_geometry.png` / `.svg` | 横轴为交会角 γ，纵轴为测站到源距离 d，颜色为定位区域直径；红虚线为各距离下的离散最优交会角 | `geometry_heatmap`、`plot_heatmap`；对称双站理论计算 | 支持交会角接近 90°、距离增大导致定位区域变大的几何规律，并可与闭式表达式交叉验证 | 问题一/二几何规律与模型依据 | 色标截断到有限值的 98% 分位；网格范围和示向误差固定，不能解释为任意场景的精确上界 |
| `curve_station_count.png` / `.svg` | 横轴为检测点个数 m，纵轴为定位区域直径（对数轴）；比较均匀布站与随机布站中位数，红线为 20 m 清除门槛 | `station_count_curve`、`plot_station_curve`；随机样本由固定种子生成 | 支持增加测站的边际收益递减，以及共线布站退化；可说明仅靠交会难以直接达到清除半径 | 问题二结果分析及问题二到三的过渡 | 随机曲线是有限次模拟的中位数，不是概率保证；m=2 均匀布局含特定退化构型 |
| `sensitivity_receive_radius.png` / `.svg` | 横轴为假设最小有效接收半径；蓝线为最坏定位直径，紫线为移动距离，使用双纵轴 | `sensitivity_receive_radius`、`plot_sensitivity`；求解配置见补充 JSON | 支持精度与移动代价的权衡及参数敏感性讨论 | 问题二敏感性分析 | 双纵轴仅用于趋势展示，不能直接比较两条曲线的数值大小；结论依赖所扫描半径和离散网格 |

重绘可使用 `python -m codes.analysis_supplement --replot`（具体参数以脚本命令行帮助为准）。论文引用时应同时报告扫描范围、误差设定和随机种子。
