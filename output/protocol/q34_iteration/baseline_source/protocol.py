"""Robot protocol from attachment 2. Importing this module never connects."""
from __future__ import annotations
from dataclasses import dataclass, field
from http.client import HTTPException
import json
import math
from pathlib import Path
import threading
import time
import unicodedata
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, build_opener, ProxyHandler, HTTPRedirectHandler
import uuid


class ProtocolError(RuntimeError):
    pass


class ActionRejected(ProtocolError):
    def __init__(self, status, response):
        self.status, self.response = status, response
        super().__init__(f"HTTP {status}, action rejected: {response}")


class UncertainAction(ProtocolError):
    """Do not issue a new action; call retry_pending() with the retained bytes."""


def identifier(value, limit):
    if (not isinstance(value, str) or not 1 <= len(value.encode('utf-8')) <= limit
            or any(unicodedata.category(c) in {'Cc', 'Cf', 'Cs'} for c in value)):
        raise ValueError('Invalid identifier')
    return value


def number(value, limit=2_000_000):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or abs(value) > limit):
        raise ValueError('Expected a finite number in range')
    return value


def encode(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False,
                      separators=(',', ':')).encode('utf-8')


def decode(raw):
    def pairs(items):
        out = {}
        for key, value in items:
            if key in out:
                raise ValueError('Duplicate JSON key')
            out[key] = value
        return out
    def invalid(value):
        raise ValueError(f'Invalid JSON constant: {value}')
    obj = json.loads(raw.decode('utf-8'), object_pairs_hook=pairs,
                     parse_constant=invalid)
    if not isinstance(obj, dict):
        raise ValueError('Expected JSON object')
    return obj


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class HttpTransport:
    """Explicit opt-in transport. The robot API cannot identify practice/formal mode."""
    def __init__(self, base_url='http://127.0.0.1:2026', *, allow_network=False):
        if not allow_network:
            raise ValueError('HTTP disabled; use OfflineStub or explicitly allow_network=True')
        url = urlsplit(base_url)
        if (url.scheme != 'http' or url.hostname not in ('127.0.0.1', 'localhost', '::1')
                or url.username or url.password or url.path not in ('', '/')
                or url.query or url.fragment or url.port is None):
            raise ValueError('Expected an explicit loopback HTTP address and port')
        self.base_url = base_url.rstrip('/')
        self.opener = build_opener(ProxyHandler({}), NoRedirect())

    def __call__(self, path, body, timeout):
        if path not in ('/enter', '/measure', '/clear', '/exit'):
            raise ValueError('Unknown action path')
        req = Request(self.base_url + path, data=body,
                      headers={'Content-Type': 'application/json'}, method='POST')
        try:
            response = self.opener.open(req, timeout=timeout)
        except HTTPError as exc:
            response = exc
        with response:
            return response.code, response.read()


@dataclass
class RobotState:
    phase: str = 'new'
    position: tuple = (0.0, 0.0)
    channel: int = 1
    virtual_time_s: float = 0.0
    deadline: float | None = None
    max_virtual_duration_s: float = 360000.0
    cleared_channels: set = field(default_factory=set)


class RobotClient:
    def __init__(self, robot_id, transport=None, *, log_path=None,
                 attempts=3, timeout=5.0, backoff=0.1, clock=time.monotonic):
        self.robot_id = identifier(robot_id, 64)
        if attempts < 1 or timeout <= 0 or backoff < 0:
            raise ValueError('Invalid retry configuration')
        self.transport = transport  # No implicit HTTP default.
        self.state = RobotState()
        self.attempts, self.timeout, self.backoff = attempts, timeout, backoff
        self.clock = clock
        self.pending = None
        self._lock = threading.Lock()
        self.log_path = Path(log_path) if log_path is not None else None
        if self.log_path:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)

    @property
    def remaining_real_duration_s(self):
        return None if self.state.deadline is None else max(0.0, self.state.deadline - self.clock())

    def _log(self, **event):
        if self.log_path:
            with self.log_path.open('a', encoding='utf-8') as f:
                f.write(encode({'wall_time_ns': time.time_ns(), **event}).decode('utf-8') + '\n')

    def enter(self):
        return self._action('/enter')

    def measure(self, x, y, channel):
        return self._action('/measure', x, y, channel)

    def clear(self, x, y, channel):
        return self._action('/clear', x, y, channel)

    def exit(self):
        return self._action('/exit')

    def _action(self, path, x=None, y=None, channel=None):
        with self._lock:
            if self.pending is not None:
                raise UncertainAction('Resolve pending action with retry_pending() first')
            if self.transport is None:
                raise ProtocolError('No transport configured; HTTP is disabled by default')
            if (path == '/enter' and self.state.phase != 'new'
                    or path != '/enter' and self.state.phase != 'running'):
                raise ProtocolError(f'Invalid action in phase {self.state.phase}')
            self._check_deadline()
            payload = dict(arena_id='default', robot_id=self.robot_id,
                           request_id=uuid.uuid4().hex)
            if path in ('/measure', '/clear'):
                if type(channel) is not int or not 1 <= channel <= 20:
                    raise ValueError('channel must be an integer in 1..20')
                payload.update(position={'x': number(x), 'y': number(y)}, channel=channel)
            self.pending = (path, encode(payload), self.clock())
            return self._send_pending()

    def _check_deadline(self):
        if (self.state.deadline is not None and self.clock() >= self.state.deadline
                or self.state.virtual_time_s >= self.state.max_virtual_duration_s):
            self.state.phase = 'expired'
            raise ProtocolError('Session deadline reached; no further actions will be sent')

    def retry_pending(self):
        with self._lock:
            if self.pending is None:
                raise ProtocolError('No pending action')
            return self._send_pending()

    def _send_pending(self):
        path, body, started = self.pending
        for attempt in range(1, self.attempts + 1):
            self._check_deadline()
            remaining = self.remaining_real_duration_s
            timeout = self.timeout if remaining is None else min(self.timeout, remaining)
            self._log(event='request', path=path, body_utf8=body.decode('utf-8'), attempt=attempt)
            try:
                status, raw = self.transport(path, body, timeout)
            except (OSError, HTTPException, URLError) as exc:
                self._log(event='network_error', error=str(exc), path=path, attempt=attempt)
                if attempt < self.attempts:
                    time.sleep(self.backoff * 2 ** (attempt - 1))
                    continue
                raise UncertainAction('Response lost; original action retained') from exc
            self._log(event='response', path=path, status=status,
                      raw_utf8=raw.decode('utf-8', errors='replace'))
            try:
                result = decode(raw)
                if type(result.get('accepted')) is not bool:
                    raise ValueError('Missing boolean accepted')
                for key in ('real_timestamp_ms', 'virtual_time_s'):
                    number(result[key], float('inf'))
                    if result[key] < 0:
                        raise ValueError('Negative time')
                if status == 200 and result['accepted']:
                    self._validate_result(path, result)
            except (ValueError, KeyError, TypeError) as exc:
                raise UncertainAction('Invalid response; original action retained') from exc
            if status != 200 or not result['accepted']:
                # 409/429/5xx can follow a lost accepted response; never release a new action.
                if status == 200 or status in (400, 404, 405, 413, 415):
                    self.pending = None
                raise ActionRejected(status, result)
            payload = decode(body)
            self.state.virtual_time_s = float(result['virtual_time_s'])
            if path == '/enter':
                self.state.phase = 'running'
                # First attempt start is conservative even if the enter response was replayed.
                self.state.deadline = started + result['remaining_real_duration_s']
                self.state.max_virtual_duration_s = result['max_virtual_duration_s']
            elif path in ('/measure', '/clear'):
                self.state.position = (payload['position']['x'], payload['position']['y'])
                if path == '/measure':
                    self.state.channel = payload['channel']
                elif result['clear_result'] == 'success':
                    self.state.cleared_channels.add(payload['channel'])
            else:
                self.state.phase = 'exited'
            self.pending = None
            return result
        raise UncertainAction('Original action retained')

    def _validate_result(self, path, result):
        if result['virtual_time_s'] < self.state.virtual_time_s:
            raise ValueError('Virtual clock moved backwards')
        if path == '/enter':
            for key in ('remaining_real_duration_s', 'max_real_duration_s', 'max_virtual_duration_s'):
                number(result[key], float('inf'))
                if result[key] < 0:
                    raise ValueError('Negative limit')
            if not 0 <= result['remaining_real_duration_s'] <= 1200:
                raise ValueError('Invalid remaining duration')
        elif path == '/measure':
            if result['measure_result'] not in ('direction', 'near', 'no_signal'):
                raise ValueError('Unknown measure result')
            if result['measure_result'] == 'direction':
                if not 0 <= number(result['svd_deg']) < 360:
                    raise ValueError('Invalid bearing')
        elif path == '/clear':
            if result['clear_result'] not in ('success', 'no_target_in_range'):
                raise ValueError('Unknown clear result')
        elif result['exit_reason'] != 'user_exit':
            raise ValueError('Invalid exit reason')
