# -*- coding: utf-8 -*-
"""V3 策略（codes.strategy_p3v3）在本地官方同构 Engine 上的正确性测试。

覆盖三件事：全量清除（与场景真值一致）、结束原因映射、空频道覆盖证书。
全部进程内完成，不连任何服务器。
"""
import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import mock

from .benchmark_q34_local import EngineTransport, generate_scenario
from . import config as settings
from .protocol import RobotClient
from .strategy import run_strategy
from .strategy_p3v3 import solve


class V3PolicyOnEngineTests(unittest.TestCase):
    """V3 适配层端到端：本地 Engine 三种子，全部 100% 清除。"""

    def test_full_clearance_and_certificates(self):
        for seed in (1, 5, 11):
            with self.subTest(seed=seed):
                scenario = generate_scenario(3, seed)
                transport = EngineTransport(scenario)
                client = RobotClient('local-v3-test', transport)
                truth = {j['channel'] for j in scenario['jammers']}
                with TemporaryDirectory() as tmp:
                    with mock.patch.object(settings, 'PROBLEM3_OUTPUT_DIR', Path(tmp)):
                        result = run_strategy(
                            client, lambda ctx: solve(ctx), problem=3)
                summary = result['algorithm_result']
                self.assertEqual(sorted(summary['cleared_channels']), sorted(truth))
                self.assertIn(summary['stop_reason'], ('all_channels_resolved',
                                                       'cleared_limit'))
                self.assertEqual(summary['planner_errors'], 0)
                record = json.loads(Path(summary['record_path']).read_text(encoding='utf-8'))
                self.assertEqual(record['record']['inconsistent_channels'], [])
                if summary['stop_reason'] == 'all_channels_resolved':
                    certificates = record['record']['exclusion_certificates']
                    self.assertTrue(certificates)
                    for channel, certificate in certificates.items():
                        self.assertNotIn(int(channel), truth,
                                         '真实源频道不应出现不存在证书')
                        self.assertTrue(certificate['certified'],
                                        f'频道 {channel} 覆盖证书未认证: {certificate}')


if __name__ == '__main__':
    unittest.main()


class SprintFrontierEquivalenceTests(unittest.TestCase):
    """速通前沿实验的 K=16 档应与完整 V3 策略行为等价（同种子全量清除）。"""

    def test_k16_clears_all(self):
        from .sprint_frontier import SprintPolicy
        from .strategy_p3v3 import _ContextClient as _AdapterClient
        scenario = generate_scenario(3, 1)
        transport = EngineTransport(scenario)
        client = RobotClient('sprint-k16', transport)
        policy = SprintPolicy(_AdapterClient(client, []), directional=False, adaptive=True)
        policy.sprint_min_clears = 16
        client.enter()  # 正式会话由 run_strategy 进入；此处测试需手动进入
        policy.run()
        truth = {j['channel'] for j in scenario['jammers']}
        self.assertEqual(set(client.state.cleared_channels), truth)
