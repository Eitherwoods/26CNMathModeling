# 问题一图件说明

两组文件均为 PNG/SVG 同名双格式，由 `codes/problem1_solution.py` 调用 `codes/plotting.py::plot_region` 生成；输入是脚本内人工构造的确定性验证数据，不是模拟器成绩。

| 文件 | 内容与编码 | 数据来源/脚本 | 可支持结论 | 建议位置 | 使用限制 |
|---|---|---|---|---|---|
| `five_stations.png` / `.svg` | 左图为五个测站、示向中心线及 ±示向误差边界；右图为半平面交集定位区域、最远顶点对和设定真实位置 | `codes/problem1_solution.py::example_data`；结果记录 `output/Problem1/problem1_results.json` | 说明半平面交集如何形成有界定位区域，并可配合残差记录验证真实点满足约束 | 问题一模型结果或正确性验证 | 人工示例仅验证算法几何逻辑，不能外推正式演练精度或成绩 |
| `triangle.png` / `.svg` | 正三角形三站反例；右图额外画出由最远点对定义的直径圆 | `codes/problem1_solution.py::triangle_data`；同一结果 JSON | 说明“最远点对距离”不能自动替代最小包围圆直径，展示几何反例 | 问题一反例/正确性分析 | 仅为特定构造案例；直径圆是示意，不代表所有定位区域的最小包围圆 |

图中坐标单位为米，颜色和线型含义见图例与 `codes/plotting.py`。若重跑脚本，图中的小数值会随模型实现和参数变化，应同步核对 JSON 结果。
