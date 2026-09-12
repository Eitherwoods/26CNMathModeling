import unittest
import tempfile
from pathlib import Path
from .credentials import load_robot_id
from .offline_stub import OfflineStub
from .protocol import RobotClient, UncertainAction
from .strategy import load_solver, run_strategy
from .run_robot import main


class StrategyTests(unittest.TestCase):
    def test_demo_lifecycle(self):
        c = RobotClient('offline-team', OfflineStub())
        result = run_strategy(c, load_solver('codes.strategy_demo:solve'), problem=4)
        self.assertEqual(result['virtual_time_s'], 199)
        self.assertEqual(result['problem'], 4)
        self.assertEqual(c.state.phase, 'exited')

    def test_margin_exits_without_measure(self):
        stub = OfflineStub(remaining=7)
        c = RobotClient('offline-team', stub)
        result = run_strategy(c, lambda ctx: ctx.measure(0, 0, 1))
        self.assertEqual(result['reason'], 'exit_margin_reached')
        self.assertEqual(stub.executed, 2)

    def test_uncertain_action_does_not_exit(self):
        stub = OfflineStub()
        def transport(path, body, timeout):
            result = stub(path, body, timeout)
            if path == '/measure':
                raise TimeoutError('lost')
            return result
        c = RobotClient('offline-team', transport, attempts=1)
        with self.assertRaises(UncertainAction):
            run_strategy(c, lambda ctx: ctx.measure(0, 0, 1))
        self.assertEqual(stub.phase, 'running')
        self.assertIsNotNone(c.pending)

    def test_state_snapshot_and_solver_exception(self):
        c = RobotClient('offline-team', OfflineStub())
        def solver(ctx):
            ctx.state.channel = 20
            self.assertEqual(ctx.state.channel, 1)
            raise RuntimeError('algorithm failed')
        with self.assertRaises(RuntimeError): run_strategy(c, solver)
        self.assertEqual(c.state.phase, 'running')

    def test_cli_rejects_formal_and_unconfirmed_practice(self):
        for args in (['--mode', 'formal'], ['--mode', 'practice']):
            with self.assertRaises(SystemExit) as e: main(args)
            self.assertEqual(e.exception.code, 2)

    def test_team_id_file_formats_and_password_is_not_needed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'credentials.txt'
            path.write_text('参赛队号\nteam-007\n密码\nnot-used-by-program\n', encoding='utf-8')
            self.assertEqual(load_robot_id(path), 'team-007')
            path.write_text('team_no: team-008\npassword: also-not-used\n', encoding='utf-8')
            self.assertEqual(load_robot_id(path), 'team-008')
            path.write_text('密码\nnot-a-team-id\n', encoding='utf-8')
            with self.assertRaises(ValueError): load_robot_id(path)


if __name__ == '__main__':
    unittest.main()
