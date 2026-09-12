"""Run a local algorithm offline or connect to a manually started practice."""
import argparse
import json
from .credentials import load_robot_id
from .offline_stub import OfflineStub
from .protocol import RobotClient, HttpTransport
from .strategy import load_solver, run_strategy


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['offline', 'practice'], default='offline')
    parser.add_argument('--problem', type=int, choices=[3, 4], default=3)
    parser.add_argument('--strategy', default='codes.strategy_demo:solve')
    parser.add_argument('--credentials', default='tester/username-and-password.txt',
                        help='Local credential file; only the labelled team number is read')
    parser.add_argument('--base-url', default='http://127.0.0.1:2026')
    parser.add_argument('--confirm-practice', action='store_true',
                        help='User has verified the simulator UI is in practice mode')
    parser.add_argument('--log')
    args = parser.parse_args(argv)
    if args.mode == 'practice' and not args.confirm_practice:
        parser.error('practice requires --confirm-practice')
    # Load before enter so module/name mistakes cannot start a robot session.
    solver = load_solver(args.strategy)
    try:
        robot_id = 'offline-team' if args.mode == 'offline' else load_robot_id(args.credentials)
    except ValueError as exc:
        parser.error(str(exc))
    transport = (OfflineStub(robot_id=robot_id) if args.mode == 'offline' else
                 HttpTransport(args.base_url, allow_network=True))
    log = args.log or f'output/protocol/{args.mode}-p{args.problem}.jsonl'
    client = RobotClient(robot_id, transport, log_path=log)
    summary = run_strategy(client, solver, problem=args.problem)
    print(json.dumps({'mode': args.mode, 'log': log, **summary}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
