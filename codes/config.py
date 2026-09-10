# -*- coding: utf-8 -*-
"""问题一的路径、误差范围与数值精度配置。"""
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
PROBLEM1_OUTPUT_DIR = ROOT_DIR / 'output' / 'Problem1'
PROBLEM1_FIG_DIR = ROOT_DIR / 'figures' / 'Problem1'
ANGLE_ERROR_DEG = 1.0
DISTANCE_TOL = 1e-7
PARALLEL_TOL = 1e-12
DPI = 300


def fmt6(value):
    """以六位小数显示结果；机器可读文件保留原始精度。"""
    return f'{value:.6f}'
