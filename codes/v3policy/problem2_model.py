"""Q2: finite-action minimax next measurement with conservative angle bins."""
import csv
import math
from pathlib import Path
from .geometry import clip_bearing, clip_range, initial_region, diameter, minimum_enclosing_circle

def candidate_points(poly, current):
    center, radius = minimum_enclosing_circle(poly)
    far = max(((math.dist(a, b), a, b) for a in poly for b in poly), key=lambda t: t[0])
    length, a, b = far
    if length < 1e-08:
        return [center]
    ux, uy = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
    scale = min(max(radius, 20), 750)
    result = []
    for along in [-0.35, 0, 0.35]:
        for across in [-0.8, -0.5, -0.25, 0, 0.25, 0.5, 0.8]:
            q = (center[0] + scale * (along * ux - across * uy), center[1] + scale * (along * uy + across * ux))
            if max((math.dist(q, p) for p in poly)) <= 999.9 and math.dist(q, current) > 0.001:
                result.append(q)
    return result

def robust_score(poly, point, bin_width=4, error=1.005, cutoff=math.inf):
    """Upper bound on diameter for every possible reported direction.

    Reporting angle bins cover [0,360). For one bin, all true bearings lie in
    its enlarged wedge of halfwidth error+bin_width/2. Its intersection contains
    every possible posterior within that bin. Including impossible bins is safe.
    A near response already permits direct clearance; ignoring its extra distance
    information here only makes the score more conservative.
    """
    if not 0 < bin_width < 2 * (90 - error):
        raise ValueError('angle bin must produce a convex uncertainty wedge')
    bins = math.ceil(360 / bin_width)
    width = 360 / bins
    worst = 0.0
    for i in range(bins):
        posterior = clip_bearing(poly, point, (i + 0.5) * width, error + width / 2)
        if posterior:
            worst = max(worst, diameter(posterior))
            if worst > cutoff:
                return worst
    return worst

def choose_second_point(poly, current, bin_width=4, excluded=()):
    if not poly:
        raise ValueError('empty feasible region')
    candidates = [q for q in candidate_points(poly, current) if all((math.dist(q, p) > 0.001 for p in excluded))]
    if not candidates:
        raise ValueError('no range-guaranteed candidate action')
    records = []
    best = math.inf
    for q in candidates:
        score = robust_score(poly, q, bin_width)
        records.append({'x': q[0], 'y': q[1], 'worst_diameter_bound_m': score, 'move_time_s': math.dist(q, current) / 5, 'max_possible_range_m': max((math.dist(q, p) for p in poly))})
        best = min(best, score)
    eligible = [r for r in records if r['worst_diameter_bound_m'] <= best * 1.01 + 1e-07]
    selected = min(eligible, key=lambda r: r['move_time_s'])
    return ((selected['x'], selected['y']), {'selected': selected, 'candidates': records, 'angle_bin_deg': bin_width})

def run():
    out = Path(__file__).resolve().parents[2]
    table = out / 'tables'
    table.mkdir(exist_ok=True)
    poly = clip_range(clip_bearing(initial_region(), (0, 0), 0), (0, 0))
    chosen, info = choose_second_point(poly, (0, 0))
    for row in info['candidates']:
        row['selected'] = int((row['x'], row['y']) == chosen)
    with (table / 'problem2_results.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=list(info['candidates'][0]))
        writer.writeheader()
        writer.writerows(info['candidates'])
    checks = []
    for width in [8, 4, 2]:
        bound = robust_score(poly, chosen, width)
        sampled = max((diameter(clip_bearing(poly, chosen, angle / 10, 1.005)) for angle in range(3600)))
        assert bound + 1e-06 >= sampled
        checks.append({'angle_bin_deg': width, 'diameter_bound_m': bound, 'dense_angle_max_m': sampled})
    with (table / 'problem2_validation.csv').open('w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=list(checks[0]))
        w.writeheader()
        w.writerows(checks)
    return info
if __name__ == '__main__':
    import json
    print(json.dumps(run()['selected'], ensure_ascii=True))
