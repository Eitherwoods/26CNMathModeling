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

# output/ 目录约定（见 output/README.md）：ProblemN 只放官方模拟器的**在线**结果，
# 离线自检的协议日志与任务记录一律进 protocol，避免自检数字和演练成绩混在一起。
ONLINE_RECORD_DIRS = {1: PROBLEM1_OUTPUT_DIR, 2: PROBLEM2_OUTPUT_DIR,
                      3: PROBLEM3_OUTPUT_DIR, 4: PROBLEM4_OUTPUT_DIR}

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

# 问题四：以下为待演练的初始参数，不代表已经调优。
# 优化目标是虚拟时间（机器狗耗时），其中移动约占九成，故调度优先压低行进。
PROBLEM4_LATTICE_SPACING_M = 20.0
PROBLEM4_SEARCH_SPACING_M = 900.0
PROBLEM4_ORIENTATION_BINS = 24
PROBLEM4_RADIUS_BINS = 4
PROBLEM4_HYPOTHESIS_LIMIT = 96
PROBLEM4_SEARCH_INTERVAL = 4
# 已发现目标的处理是有限过程，连续完成后再恢复全域覆盖，减少跨区折返。
PROBLEM4_FINISH_DETECTED_BEFORE_SEARCH = True
PROBLEM4_TRACKING_LIMIT = 8
PROBLEM4_FAIRNESS_AGE_ROUNDS = 96
PROBLEM4_MAX_ROUNDS = 10000
PROBLEM4_INFO_THRESHOLD = 0.005
PROBLEM4_EXIT_RESERVE_S = 15.0
PROBLEM4_STEP_LENGTHS_M = (50.0, 150.0, 400.0, 800.0)

# 数值精度
DISTANCE_TOL = 1e-7
PARALLEL_TOL = 1e-12
DPI = 300
BACKEND = 'Agg'


def fmt6(value):
    """以六位小数显示结果；机器可读文件保留原始精度。"""
    return f'{value:.6f}'


def record_dir_for(problem, *, offline=False):
    """任务记录写盘目录：离线自检归 `output/protocol`，在线（演练/正式）归 `output/ProblemN`。

    目录约定见 `output/README.md`。离线桩不复刻官方误差场与判定细节，其数字不是成绩，
    因此必须与在线结果分开放，避免自检记录被误当演练结果引用。
    """
    if offline:
        return PROTOCOL_LOG_DIR
    try:
        return ONLINE_RECORD_DIRS[int(problem)]
    except (KeyError, TypeError, ValueError):
        raise ValueError(f'未知题号，无法确定记录目录：{problem!r}') from None


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
