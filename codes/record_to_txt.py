"""把协议 JSONL + 任务记录 JSON 整理成逐动作行为记录 TXT。

输出格式与既有 `output/Problem4/mission_p4_*.txt` 一致：
- 首行案例规模（如「共12个， 全向3个， 定向9个」，可省略）；
- 制表符分隔六列：接收时间、指令、位置、频道、响应摘要、虚拟时间；
- UTF-8 + CRLF。

用法（项目根目录）：
  python -m codes.record_to_txt output/protocol/practice-p4-01.jsonl ^
      --record "output/Problem4/mission_p4_20260911-203806.json" ^
      [--counts "共10（全向4，定向6）"] [--session -1] [--out 路径.txt]

JSONL 中多局追加时按 /enter→/exit 分段，--session 取第几局（-1 为最后一局）。
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path

if sys.stdout.encoding and sys.stdout.encoding.lower() not in ("utf-8", "utf8"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

COUNTS_RE = re.compile(r"共\s*(\d+)\s*（\s*全向\s*(\d+)\s*，\s*定向\s*(\d+)\s*）")


def load_sessions(jsonl_path: Path) -> list[list[dict]]:
    events = [json.loads(line) for line in
              jsonl_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    sessions: list[list[dict]] = []
    current: list[dict] | None = None
    current_enter_rid: str | None = None
    for event in events:
        kind, path = event.get("event"), event.get("path", "")
        if kind == "request" and path == "/enter":
            try:
                rid = json.loads(event.get("body_utf8", "{}")).get("request_id")
            except json.JSONDecodeError:
                rid = None
            # 同一 request_id 的重试并入当前段；不同 id 才是新的一局
            if current is None or rid != current_enter_rid:
                current = [event]
                sessions.append(current)
                current_enter_rid = rid
            else:
                current.append(event)
        elif current is not None:
            current.append(event)
            if kind == "response" and path == "/exit":
                current = None
                current_enter_rid = None
    # 丢弃只有失败 /enter、没有任何成功响应的空片段
    sessions = [s for s in sessions
                if any(e.get("event") == "response" for e in s)]
    if not sessions:
        raise SystemExit(f"{jsonl_path} 中没有找到任何有效的 /enter→/exit 会话")
    return sessions


def _wall_hms(wall_time_ns: int) -> str:
    return datetime.fromtimestamp(wall_time_ns / 1e9).strftime("%H:%M:%S")


def _parse_raw(event: dict) -> dict:
    try:
        return json.loads(event.get("raw_utf8", "{}"))
    except json.JSONDecodeError:
        return {}


def _body(event: dict) -> dict:
    try:
        return json.loads(event.get("body_utf8", "{}"))
    except json.JSONDecodeError:
        return {}


def session_to_rows(session: list[dict]) -> tuple[list[list[str]], list[str]]:
    """返回 (行列表, 警告列表)。行：[接收时间, 指令, 位置, 频道, 摘要, 虚拟时间]。"""
    rows: list[list[str]] = []
    warnings: list[str] = []
    pending: dict[str, dict] = {}
    for event in session:
        kind, path = event.get("event"), event.get("path", "")
        if kind == "request":
            if path in pending:
                warnings.append(f"{path} 出现未成对请求（重试），已覆盖")
            pending[path] = event
        elif kind == "network_error":
            warnings.append(f"{path} 网络错误: {event.get('error', '')[:80]}")
        elif kind == "response":
            req = pending.pop(path, None)
            if req is None:
                warnings.append(f"{path} 响应没有对应请求，跳过")
                continue
            body = _parse_raw(event)
            virtual = body.get("virtual_time_s", 0.0)
            vtext = f"{virtual:.3f} s"
            when = _wall_hms(event["wall_time_ns"])
            if path == "/enter":
                rows.append([when, "进入", "--", "--", "进入成功", vtext])
            elif path == "/exit":
                rows.append([when, "退出", "--", "--", "正常退出", vtext])
            elif path in ("/measure", "/clear"):
                req_body = _body(req)
                pos = req_body.get("position", {})
                where = f"({pos.get('x', 0.0):.2f}, {pos.get('y', 0.0):.2f})"
                channel = str(req_body.get("channel", "--"))
                if path == "/measure":
                    result = body.get("measure_result", "?")
                    if result == "direction":
                        summary = f"示向度 {body.get('svd_deg', 0.0):.2f}°"
                    elif result == "no_signal":
                        summary = "未检测到信号"
                    elif result == "near":
                        summary = "距离过近"
                    else:
                        summary = str(result)
                    rows.append([when, "测量", where, channel, summary, vtext])
                else:
                    result = body.get("clear_result", "?")
                    summary = {
                        "success": "清除成功",
                        "no_target_in_range": "范围内无目标",
                    }.get(result, str(result))
                    rows.append([when, "清除", where, channel, summary, vtext])
    for path in pending:
        warnings.append(f"{path} 请求无响应（连接中断），未计入")
    return rows, warnings


def build_txt(rows: list[list[str]], counts: str | None) -> str:
    lines: list[str] = []
    if counts:
        lines.append(counts)
        lines.append("")
    lines.append("接收时间\t指令\t位置\t频道\t响应摘要\t虚拟时间")
    for row in rows:
        lines.append("\t".join(row))
    return "\r\n".join(lines) + "\r\n"


def convert(jsonl: Path, record: Path | None = None, session: int = -1,
            counts: str | None = None, out: Path | None = None) -> Path:
    sessions = load_sessions(jsonl)
    chosen = sessions[session]
    rows, warnings = session_to_rows(chosen)
    for w in warnings:
        print(f"[警告] {w}")

    if record is not None:
        data = json.loads(record.read_text(encoding="utf-8"))
        steps = data.get("record", {}).get("steps", [])
        action_rows = [r for r in rows if r[1] in ("测量", "清除")]
        if len(action_rows) != len(steps):
            print(f"[警告] 行为数不一致：TXT {len(action_rows)} vs 任务记录 {len(steps)}")
        summary = data.get("summary", {})
        if counts is None and summary.get("true_total") is not None:
            counts = f"共{summary['true_total']}个"

    normalized = counts_from_text(counts) if counts else None
    text = build_txt(rows, normalized or counts)
    if out is None:
        base = record if record is not None else jsonl
        out = base.with_suffix(".txt")
    with out.open("w", encoding="utf-8", newline="") as fh:
        fh.write(text)
    print(f"[完成] {len(rows)} 行动作 → {out}")
    return out


def counts_from_text(text: str) -> str | None:
    """从模拟器历史行文字（如「共10（全向4，定向6）」）取首行规模描述。"""
    match = COUNTS_RE.search(text)
    if not match:
        return None
    total, omni, directional = match.groups()
    return f"共{total}个， 全向{omni}个， 定向{directional}个"


def main():
    parser = argparse.ArgumentParser(description="协议 JSONL → 行为记录 TXT")
    parser.add_argument("jsonl", help="协议日志（output/protocol/*.jsonl）")
    parser.add_argument("--record", help="任务记录 JSON（output/ProblemN/mission_*.json）")
    parser.add_argument("--counts", help="案例规模描述，如「共10（全向4，定向6）」")
    parser.add_argument("--session", type=int, default=-1, help="第几局，-1 为最后一局")
    parser.add_argument("--out", help="输出 TXT 路径（默认与 --record 同名 .txt）")
    args = parser.parse_args()
    convert(Path(args.jsonl), Path(args.record) if args.record else None,
            args.session, args.counts, Path(args.out) if args.out else None)


if __name__ == "__main__":
    main()
