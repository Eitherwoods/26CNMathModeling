"""Stateful policy using only public action responses, with finite coverage fallback."""
import math
from .geometry import initial_region, clip_bearing, clip_range, minimum_enclosing_circle
from .problem2_model import choose_second_point

def coverage_points(directional=False, spacing=None):
    side = spacing if spacing is not None else 900 if directional else 1500
    limit = 1800 + side
    height = side * math.sqrt(3) / 2
    n = math.ceil(limit / height) + 2
    points = []
    for j in range(-n, n + 1):
        for i in range(-n, n + 1):
            p = (side * (i + j / 2), height * j)
            if math.hypot(*p) <= limit + 1e-07:
                points.append(p)
    return points

class SearchPolicy:

    def __init__(self, client, directional=False, adaptive=True):
        self.client = client
        self.directional = directional
        self.adaptive = adaptive
        self.position = (0.0, 0.0)
        self.channel = 1
        self.cleared = set()
        self.regions = {}
        self.trace = []
        self.fallback_count = 0
        self.stop_reason = None

    def measure(self, pos, channel):
        r = self.client.measure(pos, channel)
        if r.get('accepted') is not True:
            raise RuntimeError('measurement not accepted')
        self.position = tuple(pos)
        self.channel = channel
        return r

    def clear(self, pos, channel):
        r = self.client.clear(pos, channel)
        if r.get('accepted') is not True:
            raise RuntimeError('clearance not accepted')
        self.position = tuple(pos)
        if r['clear_result'] == 'success':
            self.cleared.add(channel)
            return True
        return False

    def update(self, channel, pos, result):
        poly = self.regions.get(channel, initial_region())
        if result['measure_result'] == 'direction':
            poly = clip_range(clip_bearing(poly, pos, result['svd_deg']), pos)
            if not poly:
                raise RuntimeError('inconsistent bounded-error observations')
            self.regions[channel] = poly
        return poly

    def optical_cover(self, channel, poly):
        """Every square cell diagonal is <40m; visiting its center guarantees coverage."""
        self.fallback_count += 1
        step = 27.0
        _, a, b = max(((math.dist(a, b), a, b) for a in poly for b in poly))
        length = math.dist(a, b)
        ux, uy = ((b[0] - a[0]) / length, (b[1] - a[1]) / length) if length else (1.0, 0.0)
        transformed = [(p[0] * ux + p[1] * uy, -p[0] * uy + p[1] * ux) for p in poly]
        xmin, xmax = (min((p[0] for p in transformed)), max((p[0] for p in transformed)))
        ymin, ymax = (min((p[1] for p in transformed)), max((p[1] for p in transformed)))
        nx = max(1, math.ceil((xmax - xmin) / step))
        ny = max(1, math.ceil((ymax - ymin) / step))
        rows = []
        for j in range(ny):
            aligned = [(xmin + (i + 0.5) * (xmax - xmin) / nx, ymin + (j + 0.5) * (ymax - ymin) / ny) for i in range(nx)]
            row = [(x * ux - y * uy, x * uy + y * ux) for x, y in aligned]
            rows.extend(row if j % 2 == 0 else row[::-1])
        if math.dist(self.position, rows[-1]) < math.dist(self.position, rows[0]):
            rows.reverse()
        for q in rows:
            if self.clear(q, channel):
                return
        raise RuntimeError('optical covering exhausted without clearance; model inconsistent')

    def localize(self, channel, first, first_position=None):
        result = first
        observation_position = self.position if first_position is None else first_position
        measured_positions = [observation_position]
        for iteration in range(6 if self.adaptive else 0):
            if result['measure_result'] == 'near':
                if self.clear(observation_position, channel):
                    return
                raise RuntimeError('near response contradicted by clear')
            poly = self.update(channel, observation_position, result)
            center, radius = minimum_enclosing_circle(poly)
            self.trace.append({'channel': channel, 'iteration': iteration, 'radius_m': radius})
            if radius <= 19.99:
                if self.clear(center, channel):
                    return
                raise RuntimeError('covering-circle guarantee contradicted')
            if iteration >= 1 and radius < 90:
                if self.clear(center, channel):
                    return
            try:
                q, _ = choose_second_point(poly, self.position, bin_width=8, excluded=measured_positions)
            except ValueError:
                break
            result = self.measure(q, channel)
            observation_position = q
            measured_positions.append(q)
            if result['measure_result'] == 'no_signal':
                continue
        if result['measure_result'] == 'near':
            if self.clear(observation_position, channel):
                return
        poly = self.update(channel, observation_position, result)
        self.optical_cover(channel, poly)

    def run(self):
        entered = self.client.enter()
        if entered.get('accepted') is not True:
            raise RuntimeError('entry refused')
        pending = coverage_points(self.directional)
        while pending:
            q = min(pending, key=lambda p: math.dist(self.position, p))
            pending.remove(q)
            unresolved = [c for c in range(1, 21) if c not in self.cleared]
            if self.channel in unresolved:
                unresolved.remove(self.channel)
                unresolved.insert(0, self.channel)
            detected = []
            for channel in unresolved:
                if channel in self.cleared:
                    continue
                result = self.measure(q, channel)
                if result['measure_result'] != 'no_signal':
                    self.update(channel, q, result)
                    detected.append((channel, result))
            while detected:
                entry = min(detected, key=lambda pair: math.dist(self.position, minimum_enclosing_circle(self.regions[pair[0]])[0]) if pair[0] in self.regions else math.dist(self.position, q))
                detected.remove(entry)
                self.localize(entry[0], entry[1], q)
            if len(self.cleared) == 16:
                self.stop_reason = 'known_upper_bound_reached'
                break
        if self.stop_reason is None:
            self.stop_reason = 'all_unresolved_channels_covered'
        self.client.exit()
        return {'cleared': len(self.cleared), 'stop_reason': self.stop_reason, 'fallback_count': self.fallback_count}
if __name__ == '__main__':
    from problem3_model import run_problem
    run_problem(4)
