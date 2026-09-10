"""Small deterministic protocol double, NOT the official simulator or a scoring model."""
from __future__ import annotations
from dataclasses import dataclass
import math
import threading
import time

from .protocol import decode, encode, identifier, number


@dataclass(frozen=True)
class Source:
    channel: int
    x: float
    y: float
    radius: float = 1000.0
    direction_deg: float | None = None
    bearing_error_deg: float = 0.0


class OfflineStub:
    def __init__(self, robot_id='offline-team', sources=(), remaining=1200):
        self.robot_id = robot_id
        if type(remaining) not in (int, float) or not math.isfinite(remaining) or not 0 <= remaining <= 1200:
            raise ValueError('remaining must be a finite number in 0..1200')
        self.sources = {s.channel: s for s in sources}
        self.remaining = remaining
        self.phase = 'new'
        self.position = (0.0, 0.0)
        self.channel = 1
        self.virtual_us = 0
        self.cleared = set()
        self.cache = {}
        self.executed = 0
        self._lock = threading.Lock()

    def reply(self, status=200, accepted=False, **extra):
        return status, encode(dict(accepted=accepted, real_timestamp_ms=time.time_ns() // 1_000_000,
                                   virtual_time_s=self.virtual_us / 1e6 if accepted else 0, **extra))

    def __call__(self, path, body, timeout=5):
        if not self._lock.acquire(blocking=False):
            return self.reply(409)
        try:
            return self._handle(path, body)
        finally:
            self._lock.release()

    def _handle(self, path, body):
        if path not in ('/enter', '/measure', '/clear', '/exit'):
            return self.reply(404)
        if len(body) > 65536:
            return self.reply(413)
        try:
            payload = decode(body)
            identifier(payload['robot_id'], 64)
            identifier(payload['request_id'], 128)
            if not isinstance(payload['arena_id'], str):
                raise ValueError('Invalid arena')
            expected = {'arena_id', 'robot_id', 'request_id'}
            unknown_position = False
            if path in ('/measure', '/clear'):
                expected |= {'position', 'channel'}
                p = payload['position']
                if not isinstance(p, dict):
                    raise ValueError('Invalid position')
                number(p['x']); number(p['y'])
                ch = number(payload['channel'], 20)
                if type(payload['channel']) is not int or ch < 1:
                    raise ValueError('Invalid channel')
                unknown_position = bool(set(p) - {'x', 'y'})
        except (ValueError, KeyError, TypeError, OverflowError, RecursionError):
            return self.reply(400)
        if (set(payload) - expected or unknown_position or payload['arena_id'] != 'default'
                or payload['robot_id'] != self.robot_id):
            return self.reply()
        rid = payload['request_id']
        if rid in self.cache:
            old_path, old_payload, response = self.cache[rid]
            return response if (path, payload) == (old_path, old_payload) else self.reply(409)
        if (path == '/enter' and self.phase != 'new'
                or path != '/enter' and self.phase != 'running'):
            return self.reply()
        extra = {}
        if path == '/enter':
            self.phase = 'running'
            extra = dict(max_real_duration_s=1200, max_virtual_duration_s=360000,
                         remaining_real_duration_s=self.remaining)
        elif path == '/exit':
            self.phase = 'exited'
            extra = dict(exit_reason='user_exit')
        else:
            pos = (p['x'], p['y'])
            elapsed = math.dist(self.position, pos) / 5
            source = self.sources.get(ch) if ch not in self.cleared else None
            distance = math.dist(pos, (source.x, source.y)) if source else math.inf
            if path == '/clear':
                success = distance <= 20
                elapsed += 5 if success else 3
                if success:
                    self.cleared.add(ch)
                extra = dict(clear_result='success' if success else 'no_target_in_range')
            else:
                elapsed += 5 + int(ch != self.channel)
                self.channel = ch
                visible = source is not None and distance <= source.radius
                if visible and source.direction_deg is not None:
                    angle = math.degrees(math.atan2(pos[1] - source.y, pos[0] - source.x))
                    delta = (angle - source.direction_deg + 180) % 360 - 180
                    visible = abs(delta) <= 90
                if not visible:
                    extra = dict(measure_result='no_signal')
                elif distance <= 5:
                    extra = dict(measure_result='near')
                else:
                    bearing = math.degrees(math.atan2(source.y - pos[1], source.x - pos[0]))
                    extra = dict(measure_result='direction',
                                 svd_deg=round((bearing + source.bearing_error_deg) % 360, 2) % 360)
            self.virtual_us += round(elapsed * 1e6)
            self.position = pos
        self.executed += 1
        response = self.reply(accepted=True, **extra)
        self.cache[rid] = (path, payload, response)
        return response
