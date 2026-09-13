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

# q3/q4：以下为 2026-09-12 多轮扫档后的默认参数（离线 11 案例 −5.4%、
# 留出 7 案例 −5.8%，全部正确完成）。优化目标是虚拟时间（机器狗耗时）。
# 追踪默认值集中声明，保证两个求解器共享同一套路径约束口径。
# 这些值来自当前本地 Engine 的固定 seed 复核；仍可由 ProblemNConfig 显式覆盖。
PROBLEM3_ADAPTIVE_INITIAL_DIRECTION = True
PROBLEM3_TRACKING_PATH_LIMIT_M = 500.0
PROBLEM4_ALLOCATION_BY_PLAN = True
PROBLEM4_TRACKING_PATH_LIMIT_M = 900.0
PROBLEM4_LATTICE_SPACING_M = 20.0
PROBLEM4_SEARCH_SPACING_M = 900.0
PROBLEM4_ORIENTATION_BINS = 24
PROBLEM4_RADIUS_BINS = 4
# 2026-09-13 深夜曾采纳 96，后经顺序单线程（OMP_NUM_THREADS=1）干净对照证伪：
# pair-only 与 pair+hyp96+fair96 在确定性口径下逐位相同（24 种子均 662.0），
# 此前的"-4.7%"是并行批量基准的线程/负载伪影。维持 64。
PROBLEM4_HYPOTHESIS_LIMIT = 64
# 2026-09-13 采纳 search_interval 4→1 与 FINISH_DETECT True→False 的**成对组合**
# （本地官方同构 Engine 24 种子 732.6→696.6，留出 13-24 761.9→712.4）。
# 注意两项单独启用均无效甚至变差（interval=1 单独 = 基线；finish=False 单独
# = +11.7%），只有"搜索每轮可插空 + 追踪不再独占调度"的组合才成立，勿拆开回退。
PROBLEM4_SEARCH_INTERVAL = 1
# 已发现目标的处理是有限过程；2026-09-13 起与 search_interval=1 成对改为 False
# （证据见上），追踪与全域覆盖搜索按轮交错，跨区折返更少。
PROBLEM4_FINISH_DETECTED_BEFORE_SEARCH = False
# 离线 11 案例扫档中 2 轮优于 1、3、4、8 轮；达到上限后仍由有限后备清除
# 保证完成，因此这里只压缩启发式追踪，不改变清除证书和结束判据。
PROBLEM4_TRACKING_LIMIT = 2
# 2026-09-13 深夜曾采纳 96，与 HYPOTHESIS_LIMIT 一同被确定性对照证伪（见上），
# 维持 192。
PROBLEM4_FAIRNESS_AGE_ROUNDS = 192
# 自适应追踪轮数（2026-09-12 新方法）：按存活位置单元数缩放有效追踪上限——
# 单元很少的频道直接走后备清除（链短且每步便宜），单元很多的频道额外追加
# 追踪轮数以整片收缩区域。0 表示关闭自适应。
PROBLEM4_ADAPTIVE_UNITS_LOW = 12
PROBLEM4_ADAPTIVE_UNITS_HIGH = 25
PROBLEM4_ADAPTIVE_EXTRA_ROUNDS = 2
PROBLEM4_MAX_ROUNDS = 10000
# 已发现频道的原地补测只在信息收益足以覆盖动作开销时执行；环形布站下
# 单线程 24 种子对照将阈值由0.07调至0.09，仍保持24/24完成并再降约1.0 s/源。
PROBLEM4_INFO_THRESHOLD = 0.09
PROBLEM4_EXIT_RESERVE_S = 15.0
PROBLEM4_STEP_LENGTHS_M = (100.0, 250.0, 600.0, 1000.0)
# 搜索停靠点选法：'greedy'=最近未覆盖顶点；'tour'=预排 Hamilton 路次序。
# 只改访问顺序，不改顶点集合，覆盖证书与任意朝向判据不受影响。
PROBLEM4_SEARCH_ROUTE = 'greedy'
# 搜索布站设计：'triangular'=边长900三角格外扩一格（37点，解析构造）；
# 'optimized'=启发式布站优化（贪心删点+模拟退火，顶点更少，两项几何证书
# 由单元测试按密采样复核）。顶点数是每个待排除频道动作数的下限。
PROBLEM4_SEARCH_LAYOUT = 'rings'
# 同心环布站（2026-09-13 采纳，本地官方同构 Engine 24 种子 662.0→615.5 s/源，
# 且方向位图证书 38 洞→0 洞，严格性同时提高）。外环 12 点/1900 m 是几何地板
# （边界点覆盖顶点必在 ±33.2° 楔形内且方位空隙 ≤180°），不要减到 11 点以下。
PROBLEM4_RING_INNER = (800.0, 6)
PROBLEM4_RING_MIDDLE = (1600.0, 11)
PROBLEM4_RING_OUTER = (1900.0, 12)
PROBLEM4_RING_OFFSET_DEG = 0.0
# 联合路线：搜索顶点与清除点一起排开放路，避免"先走完搜索再回头清除"的绕行。
# 12 源局实测分离执行 27472 m，联合最优开放路 ~19500 m（2026-09-13）。
PROBLEM4_JOINT_ROUTE_WITH_CLEAR = True
# 未收缩的追踪点需要额外测向；将其提前并入路线会增加动作数，抵消移动收益。
# 仅把已进入后备清除阶段的频道并入联合路线，保持虚拟时间更低。
PROBLEM4_JOINT_ROUTE_WITH_TRACK = True
PROBLEM4_JOINT_MAX_UNITS = 24

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
