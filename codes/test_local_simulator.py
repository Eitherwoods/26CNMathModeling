"""codes.local_simulator 的正确性测试。

时序模型另有一项针对真实演练日志的重放验证（test_timing_matches_recorded_session），
若 output/protocol/ 下没有真实日志则自动跳过。
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
import unittest
from decimal import Decimal

from . import local_simulator as ls
from .protocol import HttpTransport, RobotClient


class CounterSourceTests(unittest.TestCase):
    def test_deterministic_and_label_separated(self):
        key = bytes(range(32))
        a, b = ls.CounterSource(key), ls.CounterSource(key)
        xs = [a.next("count") for _ in range(16)]
        ys = [b.next("count") for _ in range(16)]
        self.assertEqual(xs, ys)
        # 同一密钥不同流（标签）序列不同
        zs = [a.next(f"jammer/{i}/radius") for i in range(4)]
        self.assertEqual(len(set(zs)), 4)
        # 计数器状态可复核：重放第一条消息应得到相同值
        import hashlib, hmac
        msg = b"practice-case-v1\x00count\x00" + (0).to_bytes(8, "big")
        expect = int.from_bytes(hmac.new(key, msg, hashlib.sha256).digest()[:8], "big")
        self.assertEqual(xs[0], expect)

    def test_uint_n_in_range_and_rejects_bad_n(self):
        cs = ls.CounterSource(bytes(32))
        for n in (1, 2, 7, 10_000, 500_000_001):
            for _ in range(20):
                self.assertTrue(0 <= cs.uint_n("t", n) < n)
        with self.assertRaises(ValueError):
            cs.uint_n("t", 0)
        with self.assertRaises(ValueError):
            cs.uint_n("t", (1 << 63) + 1)

    def test_shuffle_is_permutation(self):
        cs = ls.CounterSource(bytes(range(32)))
        seq = list(range(1, 21))
        cs.shuffle("channels", seq)
        self.assertEqual(sorted(seq), list(range(1, 21)))


class GeneratePracticeTests(unittest.TestCase):
    def check_constraints(self, scenario, problem):
        n = scenario["jammer_count"]
        self.assertTrue(10 <= n <= 16)
        chans = [j["channel"] for j in scenario["jammers"]]
        self.assertEqual(len(set(chans)), n)
        self.assertTrue(set(chans) <= set(range(1, 21)))
        for j in scenario["jammers"]:
            x, y, r = j["x_um"], j["y_um"], j["max_receive_um"]
            self.assertTrue(x * x + y * y <= 1_770_000_000 ** 2)
            self.assertTrue(1_000_000_000 <= r <= 1_500_000_000)
            if problem == 3:
                self.assertEqual(j["kind"], "omni")
        self.assertEqual(scenario["directional_jammer_count"],
                         sum(1 for j in scenario["jammers"] if j["kind"] == "directional"))

    def test_problem3(self):
        for seed in (b"\x01" * 32, bytes(range(32)), b"\xff" * 32):
            s = ls.generate_practice(3, seed)
            self.check_constraints(s, 3)
            self.assertRegex(s["noise_seed_hex"], r"^[0-9a-f]{16}$")
        self.assertRegex(s["generator_seed_hex"], r"^[0-9a-f]{64}$")

    def test_problem4_has_directional(self):
        s = ls.generate_practice(4, bytes(range(32)))
        self.check_constraints(s, 4)
        for j in s["jammers"]:
            if j["kind"] == "directional":
                self.assertTrue(0 <= j["direction_udeg"] < 360_000_000)
            else:
                self.assertNotIn("direction_udeg", j)

    def test_rejects_other_problems(self):
        with self.assertRaises(ValueError):
            ls.generate_practice(2, bytes(32))


class NoiseTests(unittest.TestCase):
    def test_grid_value_in_unit_range(self):
        for seed in (0, 12345, 2**63):
            for ch in (0, 19):
                vals = [ls.grid_value(seed, ch, i, j)
                        for i in (-3, 0, 7) for j in (-2, 1, 9)]
                self.assertTrue(all(-1.0 <= v < 1.0 for v in vals))

    def test_error_degrees_bounded(self):
        for x, y in ((0.0, 0.0), (150.0, 150.0), (-1234.5, 987.6)):
            v = ls.error_degrees(0x1234, 3, x, y)
            self.assertTrue(-1.0 <= v < 1.0, v)

    def test_quantize_clamps_and_wraps(self):
        # 误差被限幅在真值 ±1°，且结果落在 [0, 360)
        self.assertEqual(ls.quantize_bearing_hundredths(10.0, 5.0), 1100)
        self.assertEqual(ls.quantize_bearing_hundredths(10.0, -5.0), 900)
        self.assertEqual(ls.quantize_bearing_hundredths(359.995, 0.02), 2)  # 360.015 → wrap 1.5 → 150?
        v = ls.quantize_bearing_hundredths(359.999, 0.0)
        self.assertTrue(0 <= v < 36000)


class EngineTests(unittest.TestCase):
    def make_engine(self, jammer):
        return ls.Engine({
            "noise_seed_hex": "0123456789abcdef",
            "jammers": [jammer],
        })

    def test_measure_far_is_no_signal(self):
        eng = self.make_engine({"channel": 7, "x_um": 1_400_000_000, "y_um": 0,
                                "max_receive_um": 1_000_000_000, "kind": "omni"})
        r = eng.measure(7, 0.0, 0.0)
        self.assertEqual(r["measure_result"], "no_signal")
        # 首测信道 7：5s + 换信道罚时 1s（官方 lastChannel 初始为信道 1）
        self.assertEqual(eng.t_us, 6_000_000)

    def test_measure_inside_gives_direction(self):
        eng = self.make_engine({"channel": 1, "x_um": 1_000_000_000, "y_um": 0,
                                "max_receive_um": 1_200_000_000, "kind": "omni"})
        r = eng.measure(1, 0.0, 0.0)
        self.assertEqual(r["measure_result"], "direction")
        self.assertTrue(0 <= r["svd_deg"] < 360)
        self.assertEqual(eng.t_us, 5_000_000)  # 首测信道 1：无罚时

    def test_near_within_five_meters(self):
        eng = self.make_engine({"channel": 7, "x_um": 1_000_000_000, "y_um": 0,
                                "max_receive_um": 1_200_000_000, "kind": "omni"})
        r = eng.measure(7, 996.0, 0.0)  # 距源 4 m
        self.assertEqual(r["measure_result"], "near")
        self.assertNotIn("svd_deg", r)

    def test_switch_penalty_and_last_channel(self):
        eng = self.make_engine({"channel": 1, "x_um": 10**15, "y_um": 0,
                                "max_receive_um": 1_000_000_000, "kind": "omni"})
        eng.measure(1, 0.0, 0.0)            # 首测信道1：5s（lastChannel 初始即信道1）
        eng.measure(1, 0.0, 0.0)            # 同信道：5s
        self.assertEqual(eng.t_us, 10_000_000)
        eng.measure(4, 0.0, 0.0)            # 换信道：5s+1s
        self.assertEqual(eng.t_us, 16_000_000)
        eng.measure(4, 0.0, 0.0)            # 再测同信道：5s
        self.assertEqual(eng.t_us, 21_000_000)

    def test_clear_radius_and_state(self):
        eng = self.make_engine({"channel": 7, "x_um": 10_000_000, "y_um": 0,
                                "max_receive_um": 1_000_000_000, "kind": "omni"})
        r = eng.clear(7, 0.0, 0.0)          # 距源 10 m ≤ 20 m，原地无移动
        self.assertEqual(r["clear_result"], "success")
        self.assertEqual(eng.t_us, 5_000_000)
        self.assertIn(7, eng.cleared)
        r = eng.measure(7, 0.0, 0.0)        # 已清除 → no_signal
        self.assertEqual(r["measure_result"], "no_signal")
        eng2 = self.make_engine({"channel": 7, "x_um": 50_000_000, "y_um": 0,
                                 "max_receive_um": 1_000_000_000, "kind": "omni"})
        r = eng2.clear(7, 0.0, 0.0)         # 距源 50 m > 20 m
        self.assertEqual(r["clear_result"], "no_target_in_range")
        self.assertEqual(eng2.t_us, 3_000_000)
        self.assertNotIn(7, eng2.cleared)

    def test_movement_cost(self):
        eng = self.make_engine({"channel": 1, "x_um": 10**15, "y_um": 0,
                                "max_receive_um": 1_000_000_000, "kind": "omni"})
        eng.measure(1, 500.0, 0.0)          # 100 s 移动 + 5 s 测量
        self.assertEqual(eng.t_us, 105_000_000)
        eng.measure(1, 0.0, 0.0)            # 回程 100 s + 5 s
        self.assertEqual(eng.t_us, 210_000_000)

    def test_invalid_inputs(self):
        eng = self.make_engine({"channel": 1, "x_um": 0, "y_um": 0,
                                "max_receive_um": 1_000_000_000, "kind": "omni"})
        with self.assertRaises(ls.SimError):
            eng.measure(21, 0.0, 0.0)
        with self.assertRaises(ls.SimError):
            eng.measure(1, float("nan"), 0.0)


class TimingReplayTests(unittest.TestCase):
    LOG = os.path.join("output", "protocol", "practice-p3-01.jsonl")

    def test_timing_matches_recorded_session(self):
        if not os.path.exists(self.LOG):
            self.skipTest("no recorded session log")
        events = []
        for line in open(self.LOG, encoding="utf-8"):
            e = json.loads(line)
            if e.get("event") in ("request", "response"):
                events.append(e)
        pos, t, last_ch, pending = (0.0, 0.0), Decimal(0), None, None
        ok = bad = 0
        for e in events:
            if e["event"] == "request":
                pending = json.loads(e["body_utf8"])
                continue
            if e.get("status") != 200:
                continue
            resp = json.loads(e["raw_utf8"])
            path = e["path"]
            if path == "/enter":
                t = Decimal(str(resp["virtual_time_s"]))
                continue
            if path == "/exit":
                break
            new_t = Decimal(str(resp["virtual_time_s"]))
            x, y = float(pending["position"]["x"]), float(pending["position"]["y"])
            ch = pending["channel"]
            dt_us = int((new_t - t) * Decimal(1_000_000))
            move = ls.go_round(math.hypot(x - pos[0], y - pos[1]) * 2e5)
            if path == "/measure":
                pred = move + 5_000_000 + (1_000_000 if last_ch is not None and ch - 1 != last_ch else 0)
                last_ch = ch - 1
            else:
                cost = 5_000_000 if resp.get("clear_result") == "success" else 3_000_000
                pred = move + cost
            if pred == dt_us:
                ok += 1
            else:
                bad += 1
            t, pos = new_t, (x, y)
        self.assertEqual(bad, 0, f"{bad} timing mismatches (ok={ok})")
        self.assertGreater(ok, 50)


class HttpEndToEndTests(unittest.TestCase):
    def test_full_session_over_http(self):
        scenario = ls.generate_practice(3, bytes(range(32)))
        server = ls.make_server(scenario, port=0, quiet=True)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        port = server.server_address[1]
        try:
            transport = HttpTransport(f"http://127.0.0.1:{port}", allow_network=True)
            client = RobotClient("local-test-team", transport)
            enter = client.enter()
            self.assertTrue(enter["accepted"])
            self.assertEqual(enter["max_virtual_duration_s"], 360000)
            m = client.measure(0.0, 0.0, 1)
            self.assertIn(m["measure_result"], ("direction", "near", "no_signal"))
            if m["measure_result"] == "direction":
                self.assertTrue(0 <= m["svd_deg"] < 360)
            c = client.clear(0.0, 0.0, 1)
            self.assertIn(c["clear_result"], ("success", "no_target_in_range"))
            x = client.exit()
            self.assertEqual(x["exit_reason"], "user_exit")
            with self.assertRaises(Exception):
                client.measure(0.0, 0.0, 1)  # 已退出
        finally:
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
