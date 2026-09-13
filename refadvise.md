# 三篇 bearing-only 主动定位论文的使用建议

本文档说明三篇论文如何服务于本项目 B 题，重点是问题3的建模、算法解释和论文引用。三篇论文应作为**模型设计依据和相关工作依据**，不能把论文中的定理常数直接当作本题结论。

## 1. 文献与当前模型的对应关系

| 编号 | 论文 | 最适合借鉴的部分 | 在本项目中的位置 |
| --- | --- | --- | --- |
| [1] | Vander Hook, Tokekar, Isler, ICRA 2012 | Cautious greedy 主动选点；移动成本与定位收益联合评价 | 问题2第二测站、问题3追击测站 |
| [2] | Tokekar, Isler, ICRA 2013 | 有界 bearing 误差下的传感器布置、选择和最坏情况不确定域 | 问题2鲁棒约束、问题3覆盖证书 |
| [3] | Vander Hook, Tokekar, Isler, JFR 2014 | [1] 的理论扩展、上下界和野外实验验证框架 | 问题3算法论证、复杂度与实验设计 |

当前代码已经包含这些思想的主要可迁移部分：

- [problem2_model.py](D:/数模/AAA26国赛/26CNMathModeling/codes/v3policy/problem2_model.py) 用带宽角域和最坏剩余直径计算下一测站；
- [efficient_search_v2.py](D:/数模/AAA26国赛/26CNMathModeling/codes/v3policy/efficient_search_v2.py) 同时考虑移动距离、定位不确定度和接收范围；
- [adaptive_search_v3.py](D:/数模/AAA26国赛/26CNMathModeling/codes/v3policy/adaptive_search_v3.py) 用覆盖证书约束自适应扫描点，避免因为局部贪心而漏掉未发现频道。

## 2. 推荐在问题3中采用的数学表达

### 2.1 有界误差的可能位置集

对频道 \(c\) 维护可能位置集合 \(F_c\)。在位置 \(q\) 获得 bearing 观测 \(\hat\theta\) 后，用角度误差约束更新：

\[
F_c^{+}(q)=F_c\cap W(q,\hat\theta,\alpha),
\qquad \alpha=1^\circ,
\]

其中 \(W\) 是以 \(q\) 为顶点、半角为 \(\alpha\) 的楔形区域。若没有信号，则排除接收半径之外的区域：

\[
F_c^{+}=F_c\cap\{x:\|x-q\|>R_{\min}\},
\qquad R_{\min}=1000\text{ m}.
\]

这对应 [2] 的 bounded uncertainty 思路。由于本题误差是有界的，建议在正文中优先使用“可能位置集合”“最坏直径”或“外接圆半径”，不要只报告平均定位误差。

### 2.2 安全接收约束

候选测站 \(q\) 只有在整个当前可能区域都位于可靠接收范围内时，才允许用于需要保证性的定位步骤：

\[
\max_{x\in F_c}\|q-x\|
\le R_{\min}-\varepsilon,
\qquad \varepsilon>0.
\]

当前实现使用约 \(999.9\text{ m}\) 的保守阈值。这个约束很重要：若删除它，某些角度分支可能因为实际无信号而无法更新，论文中的定位收益估计就不再成立。

### 2.3 谨慎贪心的选点目标

对每个候选点 \(q\)，预测所有可能观测分支 \(o\) 的后验集合 \(F_{c,q,o}\)，定义：

\[
J(q)=\tau(q)+\lambda\max_o D(F_{c,q,o}),
\]

其中

\[
\tau(q)=\frac{\|q-q_{\mathrm{now}}\|}{v}+t_{\mathrm{measure}}+t_{\mathrm{switch}},
\]

\(D(F)\) 可以取集合直径或最小外接圆直径。选取 \(J(q)\) 最小的候选点，即得到问题3追击阶段的 cautious greedy 规则。

本题中一次扫描点需要处理多个未解决频道，因此完整代价还应包含批量测量收益：

\[
J_{\mathrm{batch}}(q)=
\tau_{\mathrm{move}}(q)
+t_{\mathrm{scan}}(q)
+\lambda U_{\mathrm{worst}}(q)
+\mu C_{\mathrm{future}}(q).
\]

其中 \(C_{\mathrm{future}}\) 表示扫描后仍需完成的覆盖或追击代价。正文可将它解释为对 [1]、[3] 的“定位收益—运动代价”思想的本题扩展。

### 2.4 覆盖证书与终止判据

论文中的主动定位通常针对已经发现的目标；问题3还必须证明“没有遗漏频道”。因此应保留当前的全域覆盖证书：将目标圆域划分为网格单元，每个单元都由至少一个扫描锚点在 \(R_{\min}-\varepsilon\) 内覆盖。只有当所有剩余单元均已扫描，才能判定未发现频道不存在。

这个证书是本题特有的扩展，不能声称是 [1]–[3] 直接证明的结论。推荐在论文中写成：

> 借鉴有界 bearing 误差下的鲁棒传感器布置思想[2]，本文进一步针对未知频道数量建立全域覆盖证书；当所有证书单元均完成可靠接收检测后，才终止搜索。

## 3. 三篇论文分别怎么用

### [1] ICRA 2012

用于说明问题3的**主动追击顺序**。建议引用在：

1. 介绍“测量点不是预先完全固定，而是根据当前不确定区域动态选择”时；
2. 定义移动距离、测量时间和定位收益的联合评分时；
3. 解释为什么先处理预计能快速收缩不确定域的目标时。

适合迁移的思想是 cautious greedy 选点，不建议照搬其具体环境、方向天线模型或理论常数。本题有完整 \(0\)–\(360^\circ\) 方位角和 \(\pm1^\circ\) 有界误差，不能直接套用论文中针对方向歧义的结论。

### [2] ICRA 2013

用于说明问题3的**鲁棒几何约束和传感器选择**。建议引用在：

1. 将 bearing 测量表示为楔形可行域时；
2. 用面积、直径或外接圆半径衡量最坏定位不确定度时；
3. 要求候选测站对整个可能区域保持可靠接收时。

当前工作区的本地文件为 [Sensor_placement_and_selection_for_bearing_sensors_with_bounded_uncertainty.pdf](D:/数模/AAA26国赛/26CNMathModeling/solutions/Question-B/Sensor_placement_and_selection_for_bearing_sensors_with_bounded_uncertainty.pdf)。使用其中结论时，应明确注明本题采用的是“有界误差几何思想”，而不是复现其正方形区域和传感器网格的原问题。

### [3] JFR 2014

用于说明问题3的**算法验证框架**。建议借鉴以下写法：

- 把总时间拆成移动、测量、换频道和清除成本；
- 同时报告平均性能、最坏场景和成功率；
- 给出贪心策略的适用条件与局限；
- 用固定场景或固定随机种子做可复现实验。

本项目当前证据见 [p3-evidence.md](D:/数模/AAA26国赛/26CNMathModeling/solutions/evidence/p3-evidence.md)。其中 275.4 s/源、24/24 全清是本地官方同构环境的离线复刻结果，不能写成三篇论文已经证明的性能，也不能写成正式模拟器成绩。

## 4. 不建议直接照搬的内容

1. 不直接使用三篇论文中的上下界常数。论文的区域、传感器和方向观测假设与本题不同。
2. 不把主轴垂直站位作为硬约束。离线成对消融中，加入该约束后平均时间由约 271.76 s/源升至约 274.04 s/源，清除率仍为 100%。
3. 不删除全域覆盖证书。问题3的未知频道数和空频道终止判据需要它。
4. 不把有限角度采样得到的最坏值写成严格连续域定理。应称为“离散候选集上的保守上界”或“数值近似”。

## 5. 推荐的正文引用句式

可直接改写为：

> 针对有界 bearing 误差下的主动定位问题，已有研究从谨慎贪心选点、传感器布置和最坏情况不确定域三个角度进行了研究[1–3]。本文保留其“定位收益与移动代价联合评价”的思想，并将观测可行域扩展为问题3中的频道级可能位置集合。考虑到本题还存在未知频道、有限接收半径和空频道终止要求，本文另外建立全域覆盖证书，并将其作为搜索结束的必要条件。

若只引用某一篇：

- 主动选点： “参考 cautious greedy 主动定位思想[1]……”
- 有界误差： “按照 bounded uncertainty 的几何表示[2]……”
- 理论与实验： “实验设计参考其移动—测量联合成本和上下界比较框架[3]……”

## 6. GB/T 7714-2015 参考文献格式

建议采用顺序编码制，正文中按首次出现顺序标为 [1]、[2]、[3]。以下条目可直接放入中文论文参考文献表：

```text
[1] VANDER HOOK J, TOKEKAR P, ISLER V. Cautious greedy strategy for bearing-based active localization: Experiments and theoretical analysis[C]//2012 IEEE International Conference on Robotics and Automation. St. Paul, MN, USA: IEEE, 2012: 1787-1792. DOI:10.1109/ICRA.2012.6225244.

[2] TOKEKAR P, ISLER V. Sensor placement and selection for bearing sensors with bounded uncertainty[C]//2013 IEEE International Conference on Robotics and Automation. Karlsruhe, Germany: IEEE, 2013: 2515-2520. DOI:10.1109/ICRA.2013.6630920.

[3] VANDER HOOK J, TOKEKAR P, ISLER V. Cautious greedy strategy for bearing-only active localization: Analysis and field experiments[J]. Journal of Field Robotics, 2014, 31(2): 296-318. DOI:10.1002/rob.21499.
```

DOI 链接：

- [1] https://doi.org/10.1109/ICRA.2012.6225244
- [2] https://doi.org/10.1109/ICRA.2013.6630920
- [3] https://doi.org/10.1002/rob.21499

## 7. BibTeX 格式

```bibtex
@inproceedings{vanderhook2012cautious,
  author    = {Vander Hook, Joshua and Tokekar, Pratap and Isler, Volkan},
  title     = {Cautious Greedy Strategy for Bearing-Based Active Localization: Experiments and Theoretical Analysis},
  booktitle = {2012 IEEE International Conference on Robotics and Automation},
  pages     = {1787--1792},
  year      = {2012},
  publisher = {IEEE},
  doi       = {10.1109/ICRA.2012.6225244}
}

@inproceedings{tokekar2013sensor,
  author    = {Tokekar, Pratap and Isler, Volkan},
  title     = {Sensor Placement and Selection for Bearing Sensors with Bounded Uncertainty},
  booktitle = {2013 IEEE International Conference on Robotics and Automation},
  pages     = {2515--2520},
  year      = {2013},
  publisher = {IEEE},
  doi       = {10.1109/ICRA.2013.6630920}
}

@article{vanderhook2014cautious,
  author  = {Vander Hook, Joshua and Tokekar, Pratap and Isler, Volkan},
  title   = {Cautious Greedy Strategy for Bearing-Only Active Localization: Analysis and Field Experiments},
  journal = {Journal of Field Robotics},
  volume  = {31},
  number  = {2},
  pages   = {296--318},
  year    = {2014},
  doi     = {10.1002/rob.21499}
}
```

## 8. 当前论文的 LaTeX 手工引用写法

当前 [body/main.tex](D:/数模/AAA26国赛/26CNMathModeling/body/main.tex:634) 使用 `thebibliography`，因此可以直接加入以下条目：

~~~latex
\bibitem{vanderhook2012}
Vander Hook J, Tokekar P, Isler V.
Cautious greedy strategy for bearing-based active localization:
Experiments and theoretical analysis[C]//2012 IEEE International Conference
on Robotics and Automation. IEEE, 2012: 1787--1792.
DOI:10.1109/ICRA.2012.6225244.

\bibitem{tokekar2013}
Tokekar P, Isler V.
Sensor placement and selection for bearing sensors with bounded uncertainty[C]//
2013 IEEE International Conference on Robotics and Automation.
IEEE, 2013: 2515--2520.
DOI:10.1109/ICRA.2013.6630920.

\bibitem{vanderhook2014}
Vander Hook J, Tokekar P, Isler V.
Cautious greedy strategy for bearing-only active localization:
Analysis and field experiments[J].
Journal of Field Robotics, 2014, 31(2): 296--318.
DOI:10.1002/rob.21499.
~~~

正文引用示例：

~~~latex
已有研究提出了谨慎贪心主动选点方法\cite{vanderhook2012}，
并研究了有界不确定性下的 bearing 传感器布置\cite{tokekar2013}。
本文的实验设计参考了其移动与测量联合成本框架\cite{vanderhook2014}。
~~~

如果使用 BibTeX，则将上面的 `\bibitem` 改为 `references.bib` 中的 BibTeX 条目，并在正文末尾使用：

~~~latex
\bibliographystyle{gbt7714-numerical}
\bibliography{references}
~~~

二者只选一种，避免手工条目和 BibTeX 重复生成同一组文献。

## 9. 引用顺序与提交前检查

如果正文先讲主动选点，再讲有界误差，最后讲实验框架，引用顺序应为 [1]、[2]、[3]；如果先讲几何误差，应调整编号，使编号按正文首次出现顺序排列。最终提交前检查作者、题名、会议或期刊、年份、页码和 DOI 是否与 DOI 页面一致，并统一全文的英文题名大小写、标点和参考文献类型标识 `[C]`、`[J]`。

