"""Independent synthetic environment. Hidden source data never enters the policy."""
from dataclasses import dataclass
import hashlib
import math
import random
import time

@dataclass
class Source:
    channel: int
    x: float
    y: float
    radius: float
    direction: float | None
    cleared: bool = False

class LocalSimulator:

    def __init__(self, sources, seed=0, error_scale=1.0):
        self._sources = {s.channel: s for s in sources}
        self._seed = seed
        self.error_scale = error_scale
        self.position = (0.0, 0.0)
        self.channel = 1
        self.virtual_time = 0.0
        self.events = []
        self.costs = dict(move=0.0, switch=0.0, measure=0.0, optical=0.0, laser=0.0)
        self.active = False
        self.started = None

    def enter(self):
        if self.started is not None:
            raise RuntimeError('session already entered')
        self.started = time.perf_counter()
        self.active = True
        return {'accepted': True, 'virtual_time_s': 0.0, 'remaining_real_duration_s': 1200}

    def _validate(self, pos, channel):
        if not self.active:
            raise RuntimeError('inactive session')
        if len(pos) != 2 or any((not math.isfinite(x) or abs(x) > 2000000 for x in pos)):
            raise ValueError('invalid position')
        if isinstance(channel, bool) or int(channel) != channel or (not 1 <= channel <= 20):
            raise ValueError('invalid channel')

    def _move(self, pos):
        cost = math.dist(self.position, pos) / 5
        self.costs['move'] += cost
        self.virtual_time += cost
        self.position = tuple(pos)

    def _response(self, action, channel, result):
        answer = {'accepted': True, 'virtual_time_s': self.virtual_time, **result}
        self.events.append({'action': action, 'position': self.position, 'channel': channel, **answer})
        if self.virtual_time > 360000:
            raise RuntimeError('virtual duration exceeded')
        return answer

    def measure(self, pos, channel):
        self._validate(pos, channel)
        self._move(pos)
        switch = int(channel != self.channel)
        self.costs['switch'] += switch
        self.costs['measure'] += 5
        self.virtual_time += switch + 5
        self.channel = channel
        s = self._sources.get(channel)
        result = {'measure_result': 'no_signal'}
        if s is not None and (not s.cleared):
            dx, dy = (pos[0] - s.x, pos[1] - s.y)
            d = math.hypot(dx, dy)
            visible = s.direction is None or dx * math.cos(math.radians(s.direction)) + dy * math.sin(math.radians(s.direction)) >= -1e-10
            if d <= s.radius and visible:
                if d <= 5:
                    result = {'measure_result': 'near'}
                else:
                    px, py = (0.0 if value == 0 else float(value) for value in pos)
                    key = f'{self._seed}:{channel}:{px.hex()}:{py.hex()}'.encode()
                    uniform = int.from_bytes(hashlib.sha256(key).digest()[:8], 'little') / (2 ** 64 - 1)
                    angle = (math.degrees(math.atan2(-dy, -dx)) + (2 * uniform - 1) * self.error_scale) % 360
                    result = {'measure_result': 'direction', 'svd_deg': round(angle, 2) % 360}
        return self._response('measure', channel, result)

    def clear(self, pos, channel):
        self._validate(pos, channel)
        self._move(pos)
        s = self._sources.get(channel)
        success = s is not None and (not s.cleared) and (math.dist(pos, (s.x, s.y)) <= 20 + 1e-10)
        self.costs['optical'] += 3
        self.virtual_time += 3
        if success:
            s.cleared = True
            self.costs['laser'] += 2
            self.virtual_time += 2
        return self._response('clear', channel, {'clear_result': 'success' if success else 'no_target_in_range'})

    def exit(self):
        if not self.active:
            raise RuntimeError('inactive session')
        self.active = False
        return {'accepted': True, 'virtual_time_s': self.virtual_time, 'exit_reason': 'user_exit'}

    def evaluate(self):
        n = len(self._sources)
        cleared = sum((s.cleared for s in self._sources.values()))
        return {'source_count': n, 'cleared': cleared, 'clear_rate': cleared / n if n else 1, 'virtual_time_s': self.virtual_time, 'average_time_s': self.virtual_time / cleared if cleared else None, 'runtime_s': time.perf_counter() - self.started, 'actions': len(self.events), **{f'{k}_s': v for k, v in self.costs.items()}}

def make_sources(seed, count=None, directional_fraction=0.0, boundary=False, radius_min=False):
    rng = random.Random(seed)
    n = count if count is not None else rng.randint(10, 16)
    channels = rng.sample(range(1, 21), n)
    directional = set(rng.sample(channels, round(n * directional_fraction)))
    result = []
    for channel in channels:
        theta = rng.uniform(0, 2 * math.pi)
        r = 1800 if boundary else 1800 * math.sqrt(rng.random())
        heading = (math.degrees(theta) if boundary else rng.uniform(0, 360)) if channel in directional else None
        result.append(Source(channel, r * math.cos(theta), r * math.sin(theta), 1000 if radius_min else rng.uniform(1000, 1500), heading))
    return result
'Reproducible synthetic evaluation; official results are intentionally separate.'
import csv, json, time
from pathlib import Path
from .search import SearchPolicy
OUT = Path(__file__).resolve().parents[2]

def evaluate_case(problem, seed, count, directional_fraction, boundary=False, radius_min=False, adaptive=True):
    env = LocalSimulator(make_sources(seed, count, directional_fraction, boundary, radius_min), seed)
    policy = SearchPolicy(env, directional=problem == 4, adaptive=adaptive)
    outcome = policy.run()
    report = env.evaluate()
    assert abs(sum(env.costs.values()) - report['virtual_time_s']) < 1e-06
    assert report['cleared'] == count, (problem, seed, report)
    report.update(problem=problem, seed=seed, scenario='boundary_outward' if boundary else 'uniform_disk', radius_min=radius_min, directional_fraction=directional_fraction, adaptive=adaptive, **outcome)
    return (report, env.events, policy.trace)

def run_problem(problem, seeds=24):
    (OUT / 'tables').mkdir(exist_ok=True)
    (OUT / 'logs').mkdir(exist_ok=True)
    reports = []
    for j in range(seeds):
        fraction = 0 if problem == 3 else [0.25, 0.5, 0.75, 1][j % 4]
        r, events, trace = evaluate_case(problem, 202609100 + problem * 1000 + j, 10 + j % 7, fraction)
        reports.append(r)
        if j == 0:
            (OUT / 'logs' / f'problem{problem}_local_trace.json').write_text(json.dumps(events, indent=2), encoding='utf-8')
            (OUT / 'logs' / f'problem{problem}_localization_trace.json').write_text(json.dumps(trace, indent=2), encoding='utf-8')
    for j in range(4):
        r, _, _ = evaluate_case(problem, 910900 + problem * 100 + j, 10 + 2 * j, 0 if problem == 3 else 1, True, True)
        reports.append(r)
    for j in range(4):
        r, _, _ = evaluate_case(problem, 202609100 + problem * 1000 + j, 10 + j % 7, 0 if problem == 3 else [0.25, 0.5, 0.75, 1][j % 4], adaptive=False)
        reports.append(r)
    target = OUT / 'tables' / f'problem{problem}_results.csv'
    with target.open('w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=list(reports[0]))
        w.writeheader()
        w.writerows(reports)
    summary = {'problem': problem, 'cases': len(reports), 'min_clear_rate': min((r['clear_rate'] for r in reports)), 'adaptive_mean_average_s': sum((r['average_time_s'] for r in reports if r['adaptive'])) / sum((r['adaptive'] for r in reports)), 'max_runtime_s': max((r['runtime_s'] for r in reports))}
    (OUT / 'logs' / f'problem{problem}_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps(summary), flush=True)
    return reports
if __name__ == '__main__':
    run_problem(3)
