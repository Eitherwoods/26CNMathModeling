"""Question 3: complete compact coverage and travel-aware localization.

The policy uses only public observations. No source count or source coordinates
from the evaluator are available here. Question 4 retains its original policy.
"""
import math
import itertools
from .geometry import minimum_enclosing_circle
from .problem2_model import candidate_points, robust_score
from .problem4_model import SearchPolicy as OriginalPolicy
from .problem4_model import coverage_points as original_coverage


def coverage_points(directional=False, spacing=None, ring_radius=1200):
    if directional or spacing is not None:
        return original_coverage(directional, spacing)
    return [(0.0, 0.0)] + [
        (ring_radius * math.cos(i * math.pi / 3), ring_radius * math.sin(i * math.pi / 3))
        for i in range(6)
    ]


def next_cover_point(current, pending):
    if current in pending:
        return current
    # At most six points remain after the origin. Exact open-route enumeration
    # avoids nearest-neighbour end-of-route backtracking. Replan after clears.
    def route_length(route):
        return math.dist(current, route[0]) + sum(math.dist(a, b) for a, b in zip(route, route[1:]))
    return min(itertools.permutations(pending), key=route_length)[0]


def economical_point(poly, current, excluded=()):
    center, radius = minimum_enclosing_circle(poly)
    length = math.dist(current, center)
    ux, uy = ((center[0]-current[0])/length, (center[1]-current[1])/length) if length else (1., 0.)
    candidates = candidate_points(poly, current)
    for fraction in (0., .25, .5, .75, 1.):
        for lateral in (0., -40., 40., -100., 100., -200., 200.):
            candidates.append((current[0] + fraction*length*ux-lateral*uy,
                               current[1] + fraction*length*uy+lateral*ux))
    records = []
    seen = set()
    for q in candidates:
        key = tuple(round(v, 6) for v in q)
        if key in seen or any(math.dist(q, p) < .01 for p in [current, *excluded]):
            continue
        seen.add(key)
        if max(math.dist(q, p) for p in poly) > 999.9:
            continue
        diameter_bound = robust_score(poly, q, bin_width=4)
        # Metres converted to seconds: direct travel, likely onward travel,
        # and a conservative uncertainty penalty. This is a heuristic score,
        # not a claim that the expression bounds actual completion time.
        cost = (math.dist(current, q) + math.dist(q, center) + 2*diameter_bound)/5 + 6
        records.append((cost, math.dist(current, q), q))
    if not records:
        raise ValueError('no range-guaranteed candidate')
    return min(records)[2]


class EfficientPolicy(OriginalPolicy):
    share_observations = True
    plan_cover_route = False
    ring_radius = 1200
    def localize(self, channel, first, first_position=None):
        result = first
        pos = self.position if first_position is None else first_position
        measured = [pos]
        for iteration in range(6):
            if result['measure_result'] == 'near':
                if self.clear(pos, channel):
                    return
                raise RuntimeError('near response contradicted by clear')
            poly = self.update(channel, pos, result)
            center, radius = minimum_enclosing_circle(poly)
            self.trace.append(dict(channel=channel, iteration=iteration, radius_m=radius))
            if radius <= 19.99:
                if self.clear(center, channel):
                    return
                raise RuntimeError('covering-circle guarantee contradicted')
            if radius < 90 and self.clear(center, channel):
                return
            try:
                q = economical_point(poly, self.position, measured)
            except ValueError:
                break
            result = self.measure(q, channel)
            pos = q
            measured.append(q)
        if result['measure_result'] == 'near':
            if self.clear(pos, channel):
                return
            raise RuntimeError('near response contradicted by clear')
        self.optical_cover(channel, self.update(channel, pos, result))

    def run(self):
        if self.client.enter().get('accepted') is not True:
            raise RuntimeError('entry refused')
        pending = coverage_points(ring_radius=self.ring_radius)
        while pending:
            q = next_cover_point(self.position, pending) if self.plan_cover_route else min(pending, key=lambda p: math.dist(self.position, p))
            pending.remove(q)
            unresolved = [c for c in range(1, 21) if c not in self.cleared]
            if self.channel in unresolved:
                unresolved.remove(self.channel)
                unresolved.insert(0, self.channel)
            detected = []
            for channel in unresolved:
                result = self.measure(q, channel)
                if result['measure_result'] != 'no_signal':
                    self.update(channel, q, result)
                    detected.append((channel, result, q))
            while detected:
                entry = min(detected, key=lambda pair: math.dist(self.position,
                    minimum_enclosing_circle(self.regions[pair[0]])[0]) if pair[0] in self.regions else math.dist(self.position, q))
                detected.remove(entry)
                self.localize(entry[0], entry[1], entry[2])
                if self.share_observations:
                    # Observe remaining known channels without moving. Retain
                    # the previous positive observation if a source is now
                    # out of reception range; no_signal never clips its region.
                    for index, (channel, previous, previous_pos) in enumerate(detected):
                        result = self.measure(self.position, channel)
                        if result['measure_result'] != 'no_signal':
                            self.update(channel, self.position, result)
                            detected[index] = (channel, result, self.position)
            if len(self.cleared) == 16:
                self.stop_reason = 'known_upper_bound_reached'
                break
        if self.stop_reason is None:
            self.stop_reason = 'all_unresolved_channels_covered'
        self.client.exit()
        return dict(cleared=len(self.cleared), stop_reason=self.stop_reason, fallback_count=self.fallback_count)
