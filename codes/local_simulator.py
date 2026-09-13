"""官方演练模拟器（jammers-simulator.exe）的本地忠实复刻。

算法与常数全部反编译自官方程序（Go 符号表完整保留），对应关系：
- scenario.(*counterSource)          HMAC-SHA256 计数器随机源（本文件 CounterSource）
- scenario.GeneratePractice          演练场景生成（本文件 generate_practice）
- bearingnoise.grid/ErrorDegrees     BLAKE2b-64 值噪声 + smoothstep 双线性插值
- bearingnoise.QuantizeBearingHundredths  示向度按 1/100 度量化、±1° 限幅、mod 36000
- simcore.(*Engine)                  观测/清除/移动判定与虚拟时间计费
- simulation-rules-v1 / practice-gen-rules-v1  两张规则模板（常数见下）

单位约定（与官方一致）：坐标/半径用微米 um（1 m = 1e6 um），角度用微度 udeg，
虚拟时间用微秒 us（1 s = 1e6 us）；移动速度 5 um/us = 5 m/s。

已知与官方的残余差异：Go 与 CPython 的 sin/cos/atan2 在 1 ulp 内可能不同，
极端情况下会使某个格点噪声或量化边界差一个最低位；结构、分布与计费完全一致。
"""
from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import math
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

# ---------------------------------------------------------------- 规则模板 ----
# practice-gen-rules-v1（0x140907530，0x70 字节）
GEN_RULES = {
    "schema": "practice-gen-rules-v1",
    "field_1800m_um": 1_800_000_000,
    "jammer_disk_radius_um": 1_800_000_000,   # 落点均匀圆盘（uniform_disk_area）；2026-09-13 修正：题目目标区域半径为 1800 m
    "field_30m_um": 30_000_000,
    "field_335_614m_um": 335_613_962,
    "receive_min_um": 1_000_000_000,          # 接收半径下限（uniform_integer_um）
    "receive_max_um": 1_500_000_000,          # 接收半径上限
    "placement_model": "uniform_disk_area",
    "receive_model": "uniform_integer_um",
    "directional_count_mode": "uniform_integer_one_to_count",
}
# simulation-rules-v1（0x140907ea0，0x80 字节）
SIM_RULES = {
    "schema": "simulation-rules-v1",
    "field_1800m_um": 1_800_000_000,
    "near_radius_um": 5_000_000,              # ≤5 m 且波束内 → measure_result="near"
    "clear_radius_um": 20_000_000,            # clear 成功判定半径
    "speed_um_per_us": 5_000_000,             # 5 m/s
    "channel_switch_us": 1_000_000,           # measure 换信道罚时
    "measure_us": 5_000_000,
    "clear_ok_us": 5_000_000,
    "clear_miss_us": 3_000_000,
    "max_virtual_us": 360_000_000_000,        # /enter 的 max_virtual_duration_s=360000
    "field_180s_us": 180_000_000,
    "bearing_model": "spatial-bearing-v1",
    "noise_grid_um": 150_000_000,             # 值噪声格距 150 m
    "field_1m_um": 1_000_000,
}

TWO64 = 1 << 64
TWO_POW_53 = 2.0 ** -53
TWO_PI = 2.0 * math.pi
ARENA_BOUND_M = 2_000_000.0                # 请求坐标合法性上限（NaN/Inf 一律拒绝）
CHANNELS = list(range(1, 21))


# ------------------------------------------------------- counterSource RNG ----
def go_round(x: float) -> int:
    """Go math.Round：四舍五入，半值远离零。"""
    if x >= 0.0:
        return int(math.floor(x + 0.5))
    return int(math.ceil(x - 0.5))


class CounterSource:
    """scenario.(*counterSource)：HMAC-SHA256(key, "practice-case-v1\\0"+label+"\\0"+be64(count)) 取前 8 字节。"""

    def __init__(self, key: bytes):
        if len(key) != 32:
            raise ValueError("counterSource key must be 32 bytes")
        self.key = key
        self.counters: dict[str, int] = {}

    def next(self, label: str) -> int:
        c = self.counters.get(label, 0)
        self.counters[label] = c + 1
        msg = b"practice-case-v1\x00" + label.encode("utf-8") + b"\x00" + c.to_bytes(8, "big")
        digest = hmac.new(self.key, msg, hashlib.sha256).digest()
        return int.from_bytes(digest[:8], "big")

    def uint_n(self, label: str, n: int) -> int:
        """无偏取模：拒绝 v < (2^64 mod n) 后返回 v % n（每次重抽同样消耗计数器）。"""
        if n <= 0 or n > (1 << 63):
            raise ValueError("uintN: n out of range")
        threshold = (TWO64 - n) % n
        v = self.next(label)
        while v < threshold:
            v = self.next(label)
        return v % n

    def shuffle(self, label: str, seq: list) -> list:
        """Fisher–Yates：for i := len-1; i > 0; i-- { j := uintN(label, i+1); swap }。"""
        for i in range(len(seq) - 1, 0, -1):
            j = self.uint_n(label, i + 1)
            seq[i], seq[j] = seq[j], seq[i]
        return seq


# ------------------------------------------------------------- 场景生成 ----
def _inside_jammer_disk(x_um: int, y_um: int) -> bool:
    """官方用 math/big 精确整数：|x|^2+|y|^2 <= 1800000000^2（题目目标区域半径 1800 m）。"""
    return abs(x_um) * abs(x_um) + abs(y_um) * abs(y_um) <= 1_800_000_000 ** 2


def generate_practice(problem: int, key: bytes) -> dict:
    """scenario.GeneratePractice：problem=3/4；返回与官方同构的场景 dict。"""
    if problem not in (3, 4):
        raise ValueError("problem must be 3 or 4")
    cs = CounterSource(key)

    n_jammers = cs.uint_n("count", 7) + 10                 # [10,16]
    shuffled = cs.shuffle("channels", list(CHANNELS))      # Fisher–Yates 1..20
    selected = sorted(shuffled[:n_jammers])                # 取前 N 后升序分配

    directional: set[int] = set()
    if problem == 4:
        dcount = cs.uint_n("directional-count", n_jammers)
        pool = cs.shuffle("directional-channels", list(selected))
        directional = set(pool[:dcount])

    jammers = []
    for ch in selected:
        while True:  # 官方落点拒绝采样：半径/角度成对重抽
            v = cs.next(f"jammer/{ch}/radius")
            r = 1_800_000_000.0 * math.sqrt((v >> 11) * TWO_POW_53)
            v = cs.next(f"jammer/{ch}/theta")
            ang = ((v >> 11) * TWO_POW_53) * TWO_PI
            x_um = go_round(math.cos(ang) * r)
            y_um = go_round(math.sin(ang) * r)
            if _inside_jammer_disk(x_um, y_um):
                break
        max_receive_um = cs.uint_n(f"jammer/{ch}/receive", 500_000_000) + 1_000_000_000
        jammer = {
            "channel": ch,
            "x_um": x_um,
            "y_um": y_um,
            "max_receive_um": max_receive_um,
            "kind": "directional" if ch in directional else "omni",
        }
        if jammer["kind"] == "directional":
            jammer["direction_udeg"] = cs.uint_n(f"jammer/{ch}/direction", 360_000_000)
        jammers.append(jammer)

    noise_seed = cs.next("noise-seed")
    return {
        "schema": "scenario-v1",
        "problem": problem,
        "generator": "practice-gen-v1",
        "generator_seed_hex": key.hex(),
        "noise_seed_hex": f"{noise_seed:016x}",
        "gen_rules": GEN_RULES,
        "sim_rules": SIM_RULES,
        "jammer_count": len(jammers),
        "directional_jammer_count": sum(1 for j in jammers if j["kind"] == "directional"),
        "jammers": jammers,
    }


# --------------------------------------------------------------- 测向噪声 ----
def _grid_u64(seed: int, ch: int, i: int, j: int) -> int:
    """bearingnoise.grid：BLAKE2b 摘要 8 字节，"%d:%d:%d:%d" % (seed, ch, i, j)。"""
    msg = f"{seed}:{ch}:{i}:{j}".encode("ascii")
    return int.from_bytes(hashlib.blake2b(msg, digest_size=8).digest(), "big")


def grid_value(seed: int, ch: int, i: int, j: int) -> float:
    """返回 [-1, 1)：u = BE64(digest)/2^64，2u-1。"""
    u = _grid_u64(seed, ch, i, j) / float(TWO64)
    return 2.0 * u - 1.0


def _smooth(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)


def error_degrees(seed: int, ch: int, x_m: float, y_m: float) -> float:
    """bearingnoise.ErrorDegrees：150 m 格距值噪声 + smoothstep 双线性插值，范围约 [-1,1)。"""
    gx = x_m / 150.0
    gy = y_m / 150.0
    ix = int(math.floor(gx))
    iy = int(math.floor(gy))
    fx = min(max(gx - ix, 0.0), 1.0)
    fy = min(max(gy - iy, 0.0), 1.0)
    wx = _smooth(fx)
    wy = _smooth(fy)
    g00 = grid_value(seed, ch, ix, iy)
    g10 = grid_value(seed, ch, ix + 1, iy)
    g01 = grid_value(seed, ch, ix, iy + 1)
    g11 = grid_value(seed, ch, ix + 1, iy + 1)
    bottom = g00 + (g10 - g00) * wx
    top = g01 + (g11 - g01) * wx
    return bottom + (top - bottom) * wy


def quantize_bearing_hundredths(bearing_deg: float, err_deg: float) -> int:
    """bearingnoise.QuantizeBearingHundredths：测量角 = true+err 四舍五入到 1/100 度，
    限幅在真值 ±1°，再 mod 36000（单位 0.01°，结果 [0,35999]）。"""
    total = (bearing_deg + err_deg) * 100.0
    s = go_round(total)
    lo = int(math.ceil((bearing_deg - 1.0) * 100.0))
    hi = int(math.floor((bearing_deg + 1.0) * 100.0))
    s = max(s, lo)
    s = min(s, hi)
    return s % 36000


def normalize_degrees(deg: float) -> float:
    """simcore.normalizeDegrees：归一到 [0,360)。"""
    return deg - 360.0 * math.floor(deg / 360.0)


def directional_covered(kind: str, jx_m: float, jy_m: float, x_m: float, y_m: float,
                        direction_udeg: int | None) -> bool:
    """simcore.directionalCoverage：omni 恒可见；directional 为朝向半平面（±90°）。"""
    if kind == "omni":
        return True
    if direction_udeg is None:
        return True
    angle = normalize_degrees(math.atan2(jy_m - y_m, jx_m - x_m) * 180.0 / math.pi)
    heading = direction_udeg / 1_000_000.0
    diff = math.remainder(angle - heading, 360.0)
    return abs(diff) <= 90.000000001


# ------------------------------------------------------------- 仿真引擎 ----
class SimError(Exception):
    def __init__(self, status: int, code: str):
        super().__init__(code)
        self.status = status
        self.code = code


class Engine:
    """simcore.(*Engine)：一次演练会话的判定与虚拟时间计费。"""

    def __init__(self, scenario: dict):
        self.scenario = scenario
        self.noise_seed = int(scenario["noise_seed_hex"], 16)
        self.jammers = {j["channel"]: j for j in scenario["jammers"]}
        self.pos = (0.0, 0.0)
        self.t_us = 0
        self.last_ch0 = 0            # 0 基"上一测量信道"；初值 0 等价官方 lastChannel=信道1（首测信道1无罚时）
        self.cleared: set[int] = set()
        self.entered = False
        self.exited = False
        self.stop_reason: str | None = None

    # -- 内部工具 --
    def _move_us(self, x: float, y: float) -> int:
        # 官方 moveTo 先把 ±0 归一为 +0，再按 hypot(delta)*1e12/5e6 四舍五入到微秒
        if x == 0.0:
            x = 0.0
        if y == 0.0:
            y = 0.0
        dist = math.hypot(x - self.pos[0], y - self.pos[1])
        return go_round(dist * 1e12 / SIM_RULES["speed_um_per_us"])

    def _charge(self, delta_us: int) -> None:
        self.t_us += delta_us
        if self.t_us >= SIM_RULES["max_virtual_us"] and self.stop_reason is None:
            self.stop_reason = "virtual_timeout"

    def _validate(self, channel, x, y) -> int:
        if type(channel) is not int or not 1 <= channel <= 20:
            raise SimError(400, "invalid_channel")
        for v in (x, y):
            if type(v) not in (int, float) or isinstance(v, bool) or not math.isfinite(v) \
                    or abs(v) > ARENA_BOUND_M:
                raise SimError(400, "invalid_coordinate")
        return channel

    # -- 动作 --
    def measure(self, channel: int, x: float, y: float) -> dict:
        ch = self._validate(channel, x, y)
        self._charge(self._move_us(x, y))
        self.pos = (float(x), float(y))
        ch0 = ch - 1
        if ch0 != self.last_ch0:
            self._charge(SIM_RULES["channel_switch_us"])
            self.last_ch0 = ch0
        self._charge(SIM_RULES["measure_us"])

        result = {"measure_result": "no_signal"}
        jam = self.jammers.get(ch)
        if jam is not None and ch not in self.cleared:
            jx = jam["x_um"] / 1e6
            jy = jam["y_um"] / 1e6
            d = math.hypot(jx - x, jy - y)
            if d <= jam["max_receive_um"] / 1e6 and \
                    directional_covered(jam["kind"], jx, jy, x, y, jam.get("direction_udeg")):
                if d <= SIM_RULES["near_radius_um"] / 1e6:
                    result = {"measure_result": "near"}
                else:
                    bearing = normalize_degrees(math.atan2(jy - y, jx - x) * 180.0 / math.pi)
                    err = error_degrees(self.noise_seed, ch0, x, y)
                    hundredths = quantize_bearing_hundredths(bearing, err)
                    result = {"measure_result": "direction", "svd_deg": hundredths / 100.0}
        return result

    def clear(self, channel: int, x: float, y: float) -> dict:
        ch = self._validate(channel, x, y)
        self._charge(self._move_us(x, y))
        self.pos = (float(x), float(y))
        # clear 无换信道罚时，也不更新 lastChannel（与官方一致）
        hit = False
        jam = self.jammers.get(ch)
        if jam is not None and ch not in self.cleared:
            jx = jam["x_um"] / 1e6
            jy = jam["y_um"] / 1e6
            if math.hypot(jx - x, jy - y) <= SIM_RULES["clear_radius_um"] / 1e6:
                hit = True
        self._charge(SIM_RULES["clear_ok_us"] if hit else SIM_RULES["clear_miss_us"])
        if hit:
            self.cleared.add(ch)
        return {"clear_result": "success" if hit else "no_target_in_range"}

    def virtual_time_s(self):
        if self.t_us % 1_000_000 == 0:
            return self.t_us // 1_000_000
        return self.t_us / 1_000_000


# ------------------------------------------------------------ HTTP 服务 ----
def _num(value):
    if type(value) not in (int, float) or isinstance(value, bool) or not math.isfinite(value):
        raise SimError(400, "invalid_number")
    return value


class _Handler(BaseHTTPRequestHandler):
    server_version = "jammers-simulator-local/1.0"
    protocol_version = "HTTP/1.1"
    engine: Engine = None
    quiet = False

    def log_message(self, fmt, *args):  # 默认静默，--verbose 时打开
        if not self.quiet:
            super().log_message(fmt, *args)

    def _send(self, status: int, payload: dict):
        raw = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8") + b"\n"
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _body(self) -> dict:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            raise SimError(400, "bad_length")
        if length <= 0 or length > 1 << 20:
            raise SimError(400, "bad_length")
        try:
            obj = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise SimError(400, "bad_json")
        if not isinstance(obj, dict):
            raise SimError(400, "bad_object")
        return obj

    def do_POST(self):
        path = urlsplit(self.path).path
        try:
            body = self._body()
            eng = self.engine
            now_ms = int(time.time() * 1000)
            if path == "/enter":
                if eng.entered or eng.exited:
                    raise SimError(409, "already_entered")
                eng.entered = True
                self._send(200, {
                    "accepted": True, "real_timestamp_ms": now_ms,
                    "virtual_time_s": eng.virtual_time_s(),
                    "max_virtual_duration_s": SIM_RULES["max_virtual_us"] // 1_000_000,
                    "max_real_duration_s": 1200,
                    "remaining_real_duration_s": 1200,
                })
                return
            if not eng.entered or eng.exited:
                raise SimError(409, "not_running")
            if eng.stop_reason is not None:
                raise SimError(409, eng.stop_reason)
            if path == "/measure":
                channel = body.get("channel")
                pos = body.get("position")
                if not isinstance(pos, dict):
                    raise SimError(400, "bad_position")
                result = eng.measure(channel, _num(pos.get("x")), _num(pos.get("y")))
                result.update({"accepted": True, "real_timestamp_ms": now_ms,
                               "virtual_time_s": eng.virtual_time_s()})
                self._send(200, result)
            elif path == "/clear":
                channel = body.get("channel")
                pos = body.get("position")
                if not isinstance(pos, dict):
                    raise SimError(400, "bad_position")
                result = eng.clear(channel, _num(pos.get("x")), _num(pos.get("y")))
                result.update({"accepted": True, "real_timestamp_ms": now_ms,
                               "virtual_time_s": eng.virtual_time_s()})
                self._send(200, result)
            elif path == "/exit":
                eng.exited = True
                eng.stop_reason = "user_exit"
                self._send(200, {"accepted": True, "real_timestamp_ms": now_ms,
                                 "virtual_time_s": eng.virtual_time_s(),
                                 "exit_reason": "user_exit"})
            else:
                raise SimError(404, "unknown_path")
        except SimError as exc:
            self._send(exc.status, {"accepted": False, "error": exc.code})


def make_server(scenario: dict, host="127.0.0.1", port=2026, quiet=True) -> ThreadingHTTPServer:
    handler = type("BoundHandler", (_Handler,), {"engine": Engine(scenario), "quiet": quiet})
    return ThreadingHTTPServer((host, port), handler)


# ------------------------------------------------------------------ CLI ----
def main(argv=None):
    parser = argparse.ArgumentParser(description="官方演练模拟器的本地复刻（不上服务器）")
    parser.add_argument("--problem", type=int, choices=[3, 4], default=3)
    parser.add_argument("--port", type=int, default=2026)
    parser.add_argument("--key-hex", help="32 字节生成器密钥（64 位 hex）；缺省随机生成")
    parser.add_argument("--scenario-in", help="从场景 JSON 载入（可复现已保存的布局）")
    parser.add_argument("--scenario-out", default=None,
                        help="把本次场景保存到该文件（默认 output/localsim/ 下自动命名）")
    parser.add_argument("--quiet", action="store_true", help="不打印 HTTP 日志")
    args = parser.parse_args(argv)

    if args.scenario_in:
        with open(args.scenario_in, "r", encoding="utf-8") as f:
            scenario = json.load(f)
    else:
        key = bytes.fromhex(args.key_hex) if args.key_hex else os.urandom(32)
        scenario = generate_practice(args.problem, key)

    if args.scenario_out is None:
        out_dir = os.path.join("output", "localsim")
        os.makedirs(out_dir, exist_ok=True)
        args.scenario_out = os.path.join(
            out_dir, f"scenario-p{scenario['problem']}-{time.strftime('%Y%m%d-%H%M%S')}.json")
    with open(args.scenario_out, "w", encoding="utf-8") as f:
        json.dump(scenario, f, ensure_ascii=False, indent=2)

    server = make_server(scenario, port=args.port, quiet=args.quiet)
    print(f"local simulator: problem={scenario['problem']} "
          f"jammers={scenario['jammer_count']} "
          f"directional={scenario.get('directional_jammer_count', 0)} "
          f"scenario={args.scenario_out}")
    print(f"listening on http://127.0.0.1:{args.port} （Ctrl+C 结束）")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
