# -*- coding: utf-8 -*-
"""模拟器 GUI 自动演练工具（UIA 方案，2026-09-11 用户解除演练自动化禁令后新建）。

安全边界（硬编码，不可通过参数绕过）：
1. 只允许点击文本精确匹配「开始问题N演练测试」的按钮；
2. 任何名称含「正式」的按钮一律拒绝点击，正式测试完全排除在自动化之外；
3. 登录密码只从 tester/username-and-password.txt 读取、只填入界面，
   不写入日志、审计文件或命令行参数；
4. run_robot 只以 practice 模式调用，调用前校验命令含 --mode practice 与
   --confirm-practice，否则拒绝执行。

技术要点：模拟器是 WebView2 应用，Chromium 无障碍树惰性加载，附加后需先向
Chrome_RenderWidgetHostHWND 发 WM_GETOBJECT(OBJID_CLIENT) 激活，才能看到
网页控件。模拟器可由本工具代启（普通启动，无需调试参数）。

用法（项目根目录）：
  python -m codes.gui_autopilot discover            # dump 界面控件树
  python -m codes.gui_autopilot drill --problem 4   # 登录(如需)+启动演练+等接口就绪
  python -m codes.gui_autopilot run --problem 4     # 跑 run_robot practice 命令
  python -m codes.gui_autopilot full --problem 4    # 全流程（drill + run）
审计产物（步骤截图）进 output/gui_audit/。
"""

from __future__ import annotations

import argparse
import ctypes
import re
import socket
import subprocess
import sys
import time
import win32gui
from pathlib import Path

from PIL import ImageGrab
from pywinauto import Desktop

PROJECT_ROOT = Path(__file__).resolve().parent.parent
TESTER_DIR = PROJECT_ROOT / "tester"
AUDIT_DIR = PROJECT_ROOT / "output" / "gui_audit"
CREDENTIALS = TESTER_DIR / "username-and-password.txt"
SIM_EXE = TESTER_DIR / "jammers-simulator.exe"

SIM_TITLE = "无线电干扰源环境模拟器"
SIM_PORT = 2026
FORBIDDEN_TEXT = "正式"
WM_GETOBJECT = 0x003D
OBJID_CLIENT = 0xFFFFFFFC

if sys.stdout.encoding and sys.stdout.encoding.lower() not in ("utf-8", "utf8"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


class AutopilotError(RuntimeError):
    """自动演练护栏、界面附加或流程执行失败。"""

    pass


# ---------------------------------------------------------------- 窗口附加

def _main_hwnd() -> int | None:
    hits = []

    def cb(hwnd, _):
        if win32gui.GetWindowText(hwnd) == SIM_TITLE:
            hits.append(hwnd)

    win32gui.EnumWindows(cb, None)
    return hits[0] if hits else None


def _poke_accessibility(hwnd: int):
    """向 Chromium 渲染窗口发 WM_GETOBJECT，激活 WebView2 无障碍树。"""
    chrome = []

    def cb(h, _):
        if win32gui.GetClassName(h) == "Chrome_RenderWidgetHostHWND":
            chrome.append(h)

    try:
        win32gui.EnumChildWindows(hwnd, cb, None)
    except Exception:
        pass
    res = ctypes.c_ulonglong()
    for h in chrome:
        ctypes.windll.user32.SendMessageTimeoutW(
            h, WM_GETOBJECT, 0, OBJID_CLIENT, 2, 3000, ctypes.byref(res))


def attach(wait_s: float = 60.0):
    """附加模拟器窗口并激活无障碍树；必要时代启模拟器。
    窗口最小化时 WebView2 子窗口全部隐藏、无障碍树塌缩，须先恢复。"""
    deadline = time.time() + wait_s
    while time.time() < deadline:
        hwnd = _main_hwnd()
        if hwnd is None:
            if not SIM_EXE.exists():
                raise AutopilotError(f"找不到模拟器: {SIM_EXE}")
            print("[autopilot] 模拟器未运行，代启（普通启动）")
            subprocess.Popen([str(SIM_EXE)], cwd=str(TESTER_DIR))
            time.sleep(4.0)
            continue
        if win32gui.IsIconic(hwnd):
            win32gui.ShowWindow(hwnd, 9)  # SW_RESTORE
            time.sleep(1.0)
        _poke_accessibility(hwnd)
        for w in Desktop(backend="uia").windows():
            if w.window_text() == SIM_TITLE:
                time.sleep(0.8)  # 等树刷新
                return w
        time.sleep(1.5)
    raise AutopilotError("60s 内未能附加模拟器窗口")


# ---------------------------------------------------------------- 审计

class Audit:
    def __init__(self):
        AUDIT_DIR.mkdir(parents=True, exist_ok=True)
        self.stem = f"audit-{time.strftime('%Y%m%d-%H%M%S')}"
        self._n = 0

    def step(self, win, name: str, detail: dict | None = None):
        self._n += 1
        shot = AUDIT_DIR / f"{self.stem}-{self._n:02d}-{name}.png"
        try:
            ImageGrab.grab().save(shot)  # 全屏快照，足以核对点击前后状态
        except Exception:
            shot = None
        print(f"[审计 {self._n:02d}] {name} {detail or ''}"
              + (f" （截图 {shot.name}）" if shot else ""))


# ---------------------------------------------------------------- 凭据

def read_credentials() -> tuple[str, str]:
    raw = CREDENTIALS.read_bytes()
    for encoding in ("utf-8", "gbk"):
        try:
            text = raw.decode(encoding)
            break
        except UnicodeDecodeError:
            continue
    else:
        raise AutopilotError("凭据文件编码无法识别")
    team = password = None
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
    for i, ln in enumerate(lines):
        if ln.startswith("参赛队号") and i + 1 < len(lines):
            team = lines[i + 1]
        if ln.startswith("密码") and i + 1 < len(lines):
            password = lines[i + 1]
    if not team or not password:
        raise AutopilotError("凭据文件缺少参赛队号或密码")
    return team, password


# ---------------------------------------------------------------- 页面操作

def all_controls(win):
    return win.descendants()


def page_controls(win) -> list[tuple[str, str]]:
    """返回 [(control_type, name)]，覆盖窗口内全部控件。"""
    out = []
    for el in all_controls(win):
        try:
            info = el.element_info
            out.append((info.control_type, (info.name or "").strip()))
        except Exception:
            continue
    return out


def page_text(win) -> str:
    return "\n".join(name for _, name in page_controls(win))


def assert_no_forbidden_button(win, context: str):
    """主导航常驻「问题N正式测试」按钮属正常现象，只记录不阻止；
    真正的防线是点击目标必须与白名单逐字相等（见 start_drill）。"""
    for ctype, name in page_controls(win):
        if ctype == "Button" and FORBIDDEN_TEXT in name:
            print(f"[护栏提示] {context}：页面存在「{name}」（正式入口，不触碰）")


def do_login_if_needed(win, audit: Audit):
    """若在登录页则登录；返回是否执行了登录动作。"""
    names = [name for _, name in page_controls(win)]
    if "参赛队号" not in names:
        audit.step(win, "skip-login", {"reason": "未发现登录表单，判定已登录"})
        return False
    assert_no_forbidden_button(win, "登录页")
    team, password = read_credentials()
    audit.step(win, "login-page", {"team": team, "password": "***"})
    edits = [el for el in all_controls(win)
             if el.element_info.control_type == "Edit"]
    if len(edits) < 2:
        raise AutopilotError("登录输入框数量不足，请跑 discover 核对")
    edits[0].set_edit_text(team)
    edits[1].set_edit_text(password)
    time.sleep(0.5)
    audit.step(win, "credentials-filled", {"team": team, "password": "***"})
    buttons = {el.element_info.name: el for el in all_controls(win)
               if el.element_info.control_type == "Button"}
    btn = buttons.get("登录")
    if btn is None:
        raise AutopilotError("找不到「登录」按钮")
    btn.invoke()
    audit.step(win, "login-clicked")
    time.sleep(3.0)
    return True


def dismiss_announcement(win, audit: Audit):
    """关闭公告弹层（若存在）。"""
    for el in all_controls(win):
        try:
            if el.element_info.control_type == "Button" \
                    and el.element_info.name == "关闭公告":
                el.invoke()
                audit.step(win, "announcement-dismissed")
                time.sleep(1.0)
                return True
        except Exception:
            continue
    return False


def drill_button_name(problem: int) -> str:
    return f"开始问题{problem}演练测试"


def start_drill(win, audit: Audit, problem: int) -> str:
    """白名单精确匹配「开始问题N演练测试」；含「正式」的按钮绝不触碰。"""
    assert_no_forbidden_button(win, "启动演练前")
    target_name = drill_button_name(problem)
    targets = [el for el in all_controls(win)
               if el.element_info.control_type == "Button"
               and el.element_info.name == target_name]
    if len(targets) != 1:
        raise AutopilotError(
            f"应恰好命中 1 个「{target_name}」按钮，实际 {len(targets)} 个；"
            "请跑 discover 核对界面。")
    if FORBIDDEN_TEXT in target_name:  # 双保险，理论上恒假
        raise AutopilotError("白名单按钮名异常")
    state = [name for ctype, name in page_controls(win)
             if ctype == "Text" and name in ("可以开始", "进行中", "未开始")]
    targets[0].invoke()
    audit.step(win, f"drill-started-p{problem}", {"button_state_before": state})
    return state[0] if state else "unknown"


def wait_interface_ready(win, audit: Audit, timeout: float = 180.0) -> str:
    """等接口就绪：优先等页面出现「就绪」字样（覆盖5秒倒计时），
    端口开放只作兜底并追加缓冲，避免倒计时内 /enter 被中止。"""
    deadline = time.time() + timeout
    port_open_at = None
    while time.time() < deadline:
        try:
            win = attach(wait_s=5)
        except AutopilotError:
            time.sleep(2.0)
            continue
        text = page_text(win)
        if "就绪" in text:
            audit.step(win, "interface-ready", {"evidence": "页面出现「就绪」"})
            time.sleep(1.0)
            return "text"
        with socket.socket() as sk:
            sk.settimeout(1.0)
            if sk.connect_ex(("127.0.0.1", SIM_PORT)) == 0:
                if port_open_at is None:
                    port_open_at = time.time()
                elif time.time() - port_open_at >= 8.0:
                    audit.step(win, "interface-ready",
                               {"evidence": f"端口 {SIM_PORT} 已开放 8s（未见「就绪」字样）"})
                    return "port"
        time.sleep(1.5)
    raise AutopilotError(f"{timeout:.0f}s 内接口未就绪，请人工查看模拟器界面")


def build_run_command(problem: int) -> list[str]:
    """读取演练命令并按参数令牌严格校验，拒绝题号或模式混淆。"""
    cmd_file = TESTER_DIR / f"runq{problem}.txt"
    line = cmd_file.read_text(encoding="utf-8").strip()
    tokens = line.split()
    mode_positions = [i for i, token in enumerate(tokens) if token == "--mode"]
    problem_positions = [i for i, token in enumerate(tokens) if token == "--problem"]
    if (len(mode_positions) != 1 or mode_positions[0] + 1 >= len(tokens)
            or tokens[mode_positions[0] + 1] != "practice"):
        raise AutopilotError(f"{cmd_file.name} 必须且只能指定一次 --mode practice")
    if (len(problem_positions) != 1 or problem_positions[0] + 1 >= len(tokens)
            or tokens[problem_positions[0] + 1] != str(problem)):
        raise AutopilotError(f"{cmd_file.name} 的 --problem 必须严格等于 {problem}")
    if tokens.count("--confirm-practice") != 1:
        raise AutopilotError(f"{cmd_file.name} 必须且只能包含一次 --confirm-practice")
    module_positions = [i for i, token in enumerate(tokens) if token == "-m"]
    if (len(module_positions) != 1 or module_positions[0] + 1 >= len(tokens)
            or tokens[module_positions[0] + 1] != "codes.run_robot"):
        raise AutopilotError(f"{cmd_file.name} 必须且只能通过 -m codes.run_robot 执行")
    if tokens and tokens[0] in ("python", "python3", "py"):
        tokens[0] = sys.executable  # 替换而非追加，避免出现两个解释器前缀
    else:
        tokens = [sys.executable, *tokens]
    return tokens


def run_robot(problem: int):
    """执行演练命令，返回 CompletedProcess（stdout 含 record_path）。"""
    cmd = build_run_command(problem)
    print("[run_robot]", " ".join(cmd))
    result = subprocess.run(cmd, cwd=str(PROJECT_ROOT), capture_output=True,
                            text=True, encoding="utf-8", errors="replace")
    sys.stdout.write(result.stdout or "")
    if result.stderr:
        sys.stderr.write(result.stderr)
    return result


def read_latest_case_counts(problem: int) -> str | None:
    """读取问题N最新一局的案例规模描述（失败返回 None）。
    优先读会话结束页的「本次演练测试干扰源数量」（分段 Text）；
    否则回导航页刷新历史表格，读最新一行的 DataItem。"""
    from codes.record_to_txt import COUNTS_RE, counts_from_text

    def counts_from_summary_page(els) -> str | None:
        names = [(el.element_info.control_type,
                  (el.element_info.name or "").strip()) for el in els]
        for i, (ctype, name) in enumerate(names):
            if ctype == "Text" and name == "本次演练测试干扰源数量":
                nums = []
                for ctype2, name2 in names[i + 1:i + 9]:
                    if name2.isdigit():
                        nums.append(name2)
                    if len(nums) == 3:
                        break
                if len(nums) == 3:
                    total, omni, directional = nums
                    return f"共{total}个， 全向{omni}个， 定向{directional}个"
        return None

    def find_counts_in_history(els) -> str | None:
        names = [(el.element_info.control_type,
                  (el.element_info.name or "").strip()) for el in els]
        anchor = None
        for i, (ctype, name) in enumerate(names):
            if ctype == "Text" and name == f"问题{problem}演练测试":
                anchor = i
        if anchor is None:
            return None
        for ctype, name in names[anchor:anchor + 120]:
            if ctype == "DataItem" and COUNTS_RE.search(name):
                return counts_from_text(name)
        return None

    def click_nav():
        for el in all_controls(win):
            try:
                if el.element_info.control_type == "Button" \
                        and el.element_info.name == "演练测试":
                    el.invoke()
                    return True
            except Exception:
                continue
        return False

    def click_refresh_all():
        for el in all_controls(win):
            try:
                if el.element_info.control_type == "Button" \
                        and el.element_info.name == "刷新历史行为日志":
                    el.invoke()
            except Exception:
                continue

    try:
        win = attach(wait_s=10)
    except AutopilotError:
        return None
    try:
        els = all_controls(win)
        counts = counts_from_summary_page(els)
        if counts:
            return counts
        click_refresh_all()
        time.sleep(2.5)
        win = attach(wait_s=10)
        counts = find_counts_in_history(all_controls(win))
        if counts:
            return counts
        if click_nav():
            time.sleep(2.0)
            win = attach(wait_s=10)
            click_refresh_all()
            time.sleep(2.5)
            win = attach(wait_s=10)
            counts = find_counts_in_history(all_controls(win))
        return counts
    except Exception:
        return None


def post_run_txt(problem: int, run_stdout: str) -> Path | None:
    """演练结束后自动生成与任务记录同名的逐动作 TXT。失败不阻断主流程。"""
    try:
        return _post_run_txt(problem, run_stdout)
    except Exception as exc:  # noqa: BLE001
        print(f"[autopilot] TXT 生成失败（不影响演练本身）: {exc}")
        return None


def _post_run_txt(problem: int, run_stdout: str) -> Path | None:
    from codes.record_to_txt import convert

    match = re.search(r'"record_path":\s*"([^"]+)"', run_stdout)
    if not match:
        print("[autopilot] 未从输出解析到 record_path，跳过 TXT 生成")
        return None
    record = Path(match.group(1))
    line = (TESTER_DIR / f"runq{problem}.txt").read_text(encoding="utf-8").strip()
    tokens = line.split()
    jsonl = None
    if "--log" in tokens:
        jsonl = PROJECT_ROOT / tokens[tokens.index("--log") + 1]
    if jsonl is None or not jsonl.exists():
        print("[autopilot] 未找到协议 JSONL，跳过 TXT 生成")
        return None
    counts = read_latest_case_counts(problem)
    if counts:
        print("[autopilot] 案例规模:", counts)
    else:
        print("[autopilot] 未能从界面读取案例规模，TXT 首行将省略")
    out = convert(jsonl, record, -1, counts, record.with_suffix(".txt"))
    return out


# ---------------------------------------------------------------- 子命令

def cmd_discover(args):
    win = attach()
    audit = Audit()
    audit.step(win, "discover")
    print("== 控件树 ==")
    for ctype, name in page_controls(win):
        if name or ctype in ("Button", "Edit"):
            print(f"   [{ctype}] {name!r}")


def cmd_drill(args):
    win = attach()
    audit = Audit()
    do_login_if_needed(win, audit)
    win = attach()
    dismiss_announcement(win, audit)
    start_drill(win, audit, args.problem)
    wait_interface_ready(win, audit)
    print(f"演练局已就绪，可执行: python -m codes.gui_autopilot run --problem {args.problem}")


def cmd_run(args):
    result = run_robot(args.problem)
    post_run_txt(args.problem, result.stdout or "")
    if result.returncode:
        raise SystemExit(result.returncode)


def back_to_drill_list(win, audit: Audit, problem: int):
    """从会话结束页回到演练列表页，确保「开始问题N演练测试」按钮可见。"""
    target = drill_button_name(problem)
    for attempt in range(4):
        win = attach(wait_s=10)
        buttons = [el for el in all_controls(win)
                   if el.element_info.control_type == "Button"
                   and el.element_info.name == target]
        if buttons:
            return win
        # 会话页：优先「返回演练测试」；否则主导航「演练测试」
        clicked = False
        for name in ("返回演练测试", "演练测试"):
            for el in all_controls(win):
                try:
                    if el.element_info.control_type == "Button" \
                            and el.element_info.name == name:
                        el.invoke()
                        clicked = True
                        break
                except Exception:
                    continue
            if clicked:
                break
        audit.step(win, f"back-to-list-{attempt}", {"clicked": clicked})
        time.sleep(2.5)
    raise AutopilotError("无法回到演练列表页（找不到 " + target + "）")


def cmd_full(args):
    audit = Audit()
    for round_no in range(1, args.rounds + 1):
        print(f"[autopilot] ===== 第 {round_no}/{args.rounds} 局（问题{args.problem}）=====")
        win = attach()
        do_login_if_needed(win, audit)
        win = attach()
        dismiss_announcement(win, audit)
        if round_no > 1:
            win = back_to_drill_list(win, audit, args.problem)
        start_drill(win, audit, args.problem)
        wait_interface_ready(win, audit)
        result = run_robot(args.problem)
        post_run_txt(args.problem, result.stdout or "")
        if result.returncode:
            raise SystemExit(result.returncode)


def main():
    parser = argparse.ArgumentParser(description="模拟器 GUI 自动演练（仅演练，禁触正式测试）")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("discover", help="连接并列出页面控件")
    for name, fn in (("drill", cmd_drill), ("run", cmd_run), ("full", cmd_full)):
        p = sub.add_parser(name)
        p.add_argument("--problem", type=int, choices=(3, 4), required=True)
        if name == "full":
            p.add_argument("--rounds", type=int, default=1,
                           help="连续演练局数（每局结束自动开下一局）")
        p.set_defaults(fn=fn)
    args = parser.parse_args()
    if args.cmd == "discover":
        cmd_discover(args)
    else:
        args.fn(args)


if __name__ == "__main__":
    main()
