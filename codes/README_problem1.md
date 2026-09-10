# 问题一：交会区域直径

在项目根目录使用 PowerShell：

```powershell
& 'E:\Python\python.exe' -X utf8 -m codes.problem1_solution
& 'E:\Python\python.exe' -X utf8 -m unittest codes.test_problem1 -v
```

已使用现有 E:\Python 环境的 NumPy、SciPy、Matplotlib，无需安装新包。
`base_models.py` 为可复用几何接口；`config.py` 管理配置；
`problem1_solution.py` 生成原思路中的五点实例与正三角形反例；
`plotting.py` 输出 PNG 和 SVG。

自有观测保存为 UTF-8 JSON 数组，每行是 `[x米, y米, 示向度]`，运行：

```powershell
& 'E:\Python\python.exe' -X utf8 -m codes.problem1_solution --input '观测.json'
```

调用接口：`from codes.base_models import locate`，输入同上。
返回状态 `empty`、`unbounded`、`point`、`segment`、`polygon`，以及顶点、直径和最远端点。
空集直径是 `None`，无界直径是正无穷（JSON 序列化成 `"infinity"`）。
算法支持跨零度、负坐标、重复约束和退化区域。仅输入同一干扰源的有效示向读数。

默认输出为 `output/Problem1/problem1_results.json`，自有数据输出为
`output/Problem1/problem1_custom.json`；图形位于 `figures/Problem1/`。
示例输入属于人工构造数据。完整浮点角度参与计算，三位小数展示值不是计算输入。

纯角域与接收圆域的交集不是同一个模型，本程序不添加任意包围框或圆域裁剪。
距离容差为 1e-7 米，平行判定阈值为 1e-12；近乎平行的极端病态数据可能触发数值错误，
不能把这一浮点实现当成任意尺度上的精确算术证明。
