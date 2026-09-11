# -*- coding: utf-8 -*-
"""问题一至问题四的路径、物理参数与数值精度配置。

坐标单位为米，角度为自 x 轴正向逆时针旋转的度数。
题目给定常量集中在此处，问题专属常量以 PROBLEMX_ 前缀命名。
"""
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
PROBLEM1_OUTPUT_DIR = ROOT_DIR / 'output' / 'Problem1'
PROBLEM1_FIG_DIR = ROOT_DIR / 'figures' / 'Problem1'
PROBLEM2_OUTPUT_DIR = ROOT_DIR / 'output' / 'Problem2'
PROBLEM2_FIG_DIR = ROOT_DIR / 'figures' / 'Problem2'
PROBLEM3_OUTPUT_DIR = ROOT_DIR / 'output' / 'Problem3'
PROBLEM3_FIG_DIR = ROOT_DIR / 'figures' / 'Problem3'
PROBLEM4_OUTPUT_DIR = ROOT_DIR / 'output' / 'Problem4'
PROBLEM4_FIG_DIR = ROOT_DIR / 'figures' / 'Problem4'
PROTOCOL_LOG_DIR = ROOT_DIR / 'output' / 'protocol'

# 几何常量（附录1、附录2）
ANGLE_ERROR_DEG = 1.0
TARGET_RADIUS_M = 1800.0
MIN_RECEIVE_RADIUS_M = 1000.0
MAX_RECEIVE_RADIUS_M = 1500.0
NEAR_DISTANCE_M = 5.0
CLEAR_RADIUS_M = 20.0

# 任务规模（问题3、问题4）
CHANNEL_COUNT = 20
MIN_SOURCE_COUNT = 10
MAX_SOURCE_COUNT = 16

# 运动与计时（附录2(4)(5)(6)(8)）
ROBOT_SPEED_MPS = 5.0
MEASURE_TIME_S = 5.0
CHANNEL_SWITCH_TIME_S = 1.0
OPTICAL_TIME_S = 3.0
CLEAR_TIME_S = 2.0
CLEAR_FAIL_TIME_S = 3.0
REAL_BUDGET_S = 1200.0
VIRTUAL_BUDGET_S = 360000.0

# 数值精度
DISTANCE_TOL = 1e-7
PARALLEL_TOL = 1e-12
DPI = 300
BACKEND = 'Agg'


def fmt6(value):
    """以六位小数显示结果；机器可读文件保留原始精度。"""
    return f'{value:.6f}'


def setup_matplotlib(backend=BACKEND):
    """统一配置 matplotlib 后端与中文字体，返回 pyplot。"""
    import matplotlib
    matplotlib.use(backend)
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.sans-serif': ['Microsoft YaHei', 'SimHei', 'DejaVu Sans'],
                         'axes.unicode_minus': False, 'svg.fonttype': 'none',
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'legend.frameon': False, 'font.size': 9})
    return plt
