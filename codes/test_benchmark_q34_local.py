# -*- coding: utf-8 -*-
"""本地基准检查器的最小篡改与空解测试。"""
import unittest
import copy
from .benchmark_q34_local import _checks, EngineTransport, generate_scenario

class BenchmarkChecksTest(unittest.TestCase):
    """验证关键约束被检查器拒绝。"""
    def setUp(self):
        """生成确定场景和独立计时引擎。"""
        self.scenario = generate_scenario(3, 1)
        self.transport = EngineTransport(self.scenario)

    def test_empty_result_invalid(self):
        """空记录不能冒充成功结果。"""
        result = _checks({}, self.scenario, self.transport)
        self.assertFalse(result['all_constraints_ok'])

    def test_tampered_channels_invalid(self):
        """非法已清除频道必须被拒绝。"""
        result = _checks({'cleared_channels': [999]}, self.scenario, self.transport)
        self.assertFalse(result['channels_legal'])

    def test_tampered_trajectory_invalid(self):
        """非有限路径必须被拒绝。"""
        result = _checks({'cleared_channels': [], 'trajectory': [[0, 0], [float('nan'), 0]], 'steps': []}, self.scenario, self.transport)
        self.assertFalse(result['finite_coordinates'])

    def test_tampered_time_invalid(self):
        """末尾时钟必须严格对应重放结果。"""
        result = _checks({'cleared_channels': [], 'end_virtual_time_s': 99}, self.scenario, self.transport)
        self.assertFalse(result['virtual_time_match'])

    def test_first_switch_and_clear_not_switching_channel(self):
        """首测换频收费，清除另一频道不改变后续测量频道。"""
        scenario = copy.deepcopy(self.scenario)
        jammer = scenario['jammers'][0]
        jammer.update(channel=5, x_um=0, y_um=0, kind='omni')
        scenario['jammers'], scenario['jammer_count'] = [jammer], 1
        transport = EngineTransport(scenario)
        steps = []
        for kind, channel in [('measure', 5), ('clear', 1), ('measure', 5), ('clear', 5)]:
            response = getattr(transport.engine, kind)(channel, 0., 0.)
            steps.append(dict(kind=kind, channel=channel, x=0., y=0.,
                              result=response[kind + '_result'], virtual_time_s=transport.engine.virtual_time_s()))
        record = dict(steps=steps, trajectory=[[0., 0.]] * 5, cleared_channels=[5],
                      moved_distance_m=0., start_virtual_time_s=0., end_virtual_time_s=19.,
                      stop_reason='all_channels_resolved')
        self.assertTrue(_checks(record, scenario, transport)['all_constraints_ok'])
        record['end_virtual_time_s'] = 20.
        self.assertFalse(_checks(record, scenario, transport)['all_constraints_ok'])
        record['end_virtual_time_s'] = 19.
        record['steps'][1]['result'] = 'success'
        self.assertFalse(_checks(record, scenario, transport)['feedback_match'])

if __name__ == '__main__': unittest.main()
