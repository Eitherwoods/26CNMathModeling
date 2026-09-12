"""Algorithm interface shared by problem 3 and problem 4."""
from copy import deepcopy
from dataclasses import dataclass
from importlib import import_module
from typing import Callable

from .protocol import RobotClient, ProtocolError


class BudgetReached(RuntimeError):
    """Stop planning and let the runner attempt a normal exit."""


@dataclass
class StrategyContext:
    client: RobotClient
    problem: int
    exit_margin_s: float = 10.0

    @property
    def state(self):
        """Snapshot; changing it does not change the protocol state."""
        return deepcopy(self.client.state)

    @property
    def remaining_real_duration_s(self):
        return self.client.remaining_real_duration_s

    def should_stop(self):
        remaining = self.remaining_real_duration_s
        return remaining is not None and remaining <= self.exit_margin_s

    def _check_budget(self):
        if self.should_stop():
            raise BudgetReached('Reserved time for exit')

    def measure(self, x, y, channel):
        self._check_budget()
        return self.client.measure(x, y, channel)

    def clear(self, x, y, channel):
        self._check_budget()
        return self.client.clear(x, y, channel)


def load_solver(spec: str) -> Callable:
    """Load a trusted local Python callable such as codes.strategy_p3:solve."""
    module, separator, name = spec.partition(':')
    if not separator or not module or not name:
        raise ValueError('Strategy must be module:function')
    solver = getattr(import_module(module), name)
    if not callable(solver):
        raise ValueError('Strategy is not callable')
    return solver


def run_strategy(client, solver, *, problem=3, exit_margin_s=10.0):
    if problem not in (3, 4):
        raise ValueError('Only problems 3 and 4 use the robot interface')
    if exit_margin_s < 0:
        raise ValueError('Exit margin must be nonnegative')
    client.enter()
    context = StrategyContext(client, problem, exit_margin_s)
    reason = 'solver_returned'
    try:
        result = solver(context)
    except BudgetReached:
        reason, result = 'exit_margin_reached', None
    # Other exceptions propagate: do not send exit over an unresolved action.
    # The user can inspect the log and manually abort the practice in the UI.
    if client.pending is not None:
        raise ProtocolError('Strategy left an unresolved action; not sending exit')
    exit_response = client.exit()
    return {'problem': problem, 'reason': reason, 'algorithm_result': result,
            'virtual_time_s': exit_response['virtual_time_s'],
            'cleared_channels': sorted(client.state.cleared_channels),
            'exit_reason': exit_response['exit_reason']}
