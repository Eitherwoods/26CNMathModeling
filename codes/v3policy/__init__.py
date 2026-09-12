# -*- coding: utf-8 -*-
"""V3 免安装运行包策略的收编版本（问题三专用）。

来源：D:/数模/AAA26国赛/问题3优化算法V3_免安装运行包/code（2026-09-13 复核）。
仅做两处工程化改造：平面模块改为包内相对导入；search.SearchPolicy 收紧为
只允许全向源。算法本体未改动，严格性论证见包内各模块 docstring：
保守外接几何（geometry）、角域交会收缩（problem2/problem4_model）、
七锚点自适应覆盖证书（adaptive_search_v3.FlexibleCoverage）。
"""
from .search import SearchPolicy  # noqa: F401

__all__ = ['SearchPolicy']
