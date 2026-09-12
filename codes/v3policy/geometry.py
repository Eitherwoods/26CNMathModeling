"""Conservative convex geometry, Cartesian bearings (degrees CCW from +x).

Polygons are cyclic ordered vertices; clipping returns lists of (x, y).
Disk approximations are CIRCUMSCRIBED, so they never deliberately discard a
feasible point. All distances are in the coordinate system's units (metres).
"""
import math
import random
from fractions import Fraction

EPS = 1e-8


def clip_halfplane(poly, a, b, c):
    """Keep a*x+b*y <= c. Empty input/output is []; degenerate edges allowed."""
    if not len(poly):
        return []
    norm = math.hypot(a,b)
    if norm == 0:
        return list(poly) if c >= 0 else []
    a,b,c = a/norm,b/norm,c/norm
    result=[]
    previous=poly[-1]
    vp=a*previous[0]+b*previous[1]-c
    for current in poly:
        vc=a*current[0]+b*current[1]-c
        inside_p,inside_c=vp<=EPS,vc<=EPS
        if inside_p != inside_c:
            # Use the same tiny relaxed boundary in classification/intersection.
            t=(vp-EPS)/(vp-vc)
            result.append((previous[0]+t*(current[0]-previous[0]),
                           previous[1]+t*(current[1]-previous[1])))
        if inside_c:
            result.append((float(current[0]),float(current[1])))
        previous,vp=current,vc
    return result


def initial_region(radius=1800.0, sides=128, center=(0.0,0.0)):
    """Return a CCW regular polygon circumscribing the specified disk."""
    if radius < 0 or sides < 3:
        raise ValueError('radius >= 0 and sides >= 3 required')
    r=radius/math.cos(math.pi/sides)
    return [(center[0]+r*math.cos((2*i+1)*math.pi/sides),
             center[1]+r*math.sin((2*i+1)*math.pi/sides)) for i in range(sides)]


def bearing_halfplanes(position, angle_deg, error_deg=1.005):
    """Three halfplanes for a forward wedge; third also handles zero width."""
    if not 0 <= error_deg < 90:
        raise ValueError('convex wedge requires 0 <= error_deg < 90')
    lo,hi=math.radians(angle_deg-error_deg),math.radians(angle_deg+error_deg)
    mid=math.radians(angle_deg)
    normals=[(math.sin(lo),-math.cos(lo)),(-math.sin(hi),math.cos(hi)),
             (-math.cos(mid),-math.sin(mid))]
    return [(a,b,a*position[0]+b*position[1]) for a,b in normals]


def clip_bearing(poly, position, angle_deg, error_deg=1.005):
    """Intersect polygon with closed forward angular uncertainty wedge."""
    for a,b,c in bearing_halfplanes(position,angle_deg,error_deg):
        poly=clip_halfplane(poly,a,b,c)
    return poly


def clip_range(poly, position, radius=1500.0, sides=128):
    """Intersect with circumscribed range polygon (outward error r*(sec(pi/n)-1))."""
    if radius < 0 or sides < 3:
        raise ValueError('radius >= 0 and sides >= 3 required')
    for i in range(sides):
        angle=2*math.pi*i/sides
        a,b=math.cos(angle),math.sin(angle)
        poly=clip_halfplane(poly,a,b,a*position[0]+b*position[1]+radius)
        if not len(poly):
            break
    return poly


def diameter(poly):
    """Exact convex polygon diameter via all vertex pairs; empty/singleton -> 0."""
    return max((math.dist(p,q) for i,p in enumerate(poly) for q in poly[i+1:]),default=0.0)


def contains_point(poly, point, tol=1e-7):
    """Closed convex polygon membership, either orientation; empty -> False."""
    if not len(poly):
        return False
    if len(poly)==1:
        return math.dist(poly[0],point)<=tol
    signs=[]
    for p,q in zip(poly,list(poly[1:])+[poly[0]]):
        dx,dy=q[0]-p[0],q[1]-p[1]
        length=math.hypot(dx,dy)
        if length:
            signs.append((dx*(point[1]-p[1])-dy*(point[0]-p[0]))/length)
    if signs and not (min(signs)>=-tol or max(signs)<=tol):
        return False
    return (min(p[0] for p in poly)-tol <= point[0] <= max(p[0] for p in poly)+tol
            and min(p[1] for p in poly)-tol <= point[1] <= max(p[1] for p in poly)+tol)


def _pair_circle(a,b):
    center=((a[0]+b[0])/2,(a[1]+b[1])/2)
    return center,math.dist(a,b)/2


def _triple_circle(a,b,c):
    bx,by=b[0]-a[0],b[1]-a[1]
    cx,cy=c[0]-a[0],c[1]-a[1]
    d=2*(bx*cy-by*cx)
    if abs(d)<=1e-15*max(1,math.hypot(bx,by)*math.hypot(cx,cy)):
        return max((_pair_circle(a,b),_pair_circle(a,c),_pair_circle(b,c)),key=lambda x:x[1])
    b2,c2=bx*bx+by*by,cx*cx+cy*cy
    center=(a[0]+(cy*b2-by*c2)/d,a[1]+(bx*c2-cx*b2)/d)
    return center,math.dist(center,a)


def minimum_enclosing_circle(poly):
    """Return ((cx,cy), radius), deterministic randomized incremental MEC.

    Empty -> ((0,0),0), singleton -> (point,0). Radius is finally rounded
    outward to the farthest vertex to ensure floating point containment.
    """
    points=[(float(p[0]),float(p[1])) for p in poly]
    if not points:
        return ((0.0,0.0),0.0)
    random.Random(73021).shuffle(points)
    circle=(points[0],0.0)
    def outside(p):
        return math.dist(p,circle[0]) > circle[1]+1e-10
    for i,p in enumerate(points):
        if not outside(p):
            continue
        circle=(p,0.0)
        for j,q in enumerate(points[:i]):
            if not outside(q):
                continue
            circle=_pair_circle(p,q)
            for r in points[:j]:
                if outside(r):
                    circle=_triple_circle(p,q,r)
    return circle[0],max(math.dist(circle[0],p) for p in points)


def intersect_halfplanes(halfplanes):
    """Classify finite 2D halfplane intersection without an artificial box.

    Return {'status': 'empty'|'unbounded'|'bounded', 'polygon': vertices}.
    Polygon is populated only for bounded intersections (including a point or
    segment). Enumeration costs O(m^3); intended for small Q1 constraint sets.
    A feasible recession direction distinguishes unbounded regions. Predicates
    and intersections use exact rational arithmetic on decimal representations
    of input coefficients, avoiding near-parallel floating-point misclassification.
    """
    planes=[]
    for a,b,c in halfplanes:
        a,b,c=(Fraction(str(v)) for v in (a,b,c))
        if a==0 and b==0:
            if c<0:
                return {'status':'empty','polygon':[]}
        else:
            planes.append((a,b,c))
    def feasible(p):
        return all(a*p[0]+b*p[1]<=c for a,b,c in planes)
    candidates=[(Fraction(0),Fraction(0))]+[(a*c/(a*a+b*b),b*c/(a*a+b*b)) for a,b,c in planes]
    vertices=[]
    for i,(a,b,c) in enumerate(planes):
        for d,e,f in planes[i+1:]:
            det=a*e-b*d
            if det != 0:
                p=((c*e-b*f)/det,(a*f-c*d)/det)
                if feasible(p):
                    vertices.append(p)
    if not vertices and not any(feasible(p) for p in candidates):
        return {'status':'empty','polygon':[]}
    # Every nontrivial recession cone has a boundary ray parallel to some
    # constraint line. Test both directions with exact signs, never angle gaps.
    directions=[(b,-a) for a,b,c in planes]+[(-b,a) for a,b,c in planes]
    if not planes or any(all(a*x+b*y<=0 for a,b,c in planes) for x,y in directions):
        return {'status':'unbounded','polygon':[]}
    unique=[(float(x),float(y)) for x,y in set(vertices)]
    center=(sum(p[0] for p in unique)/len(unique),sum(p[1] for p in unique)/len(unique))
    unique.sort(key=lambda p: math.atan2(p[1]-center[1],p[0]-center[0]))
    return {'status':'bounded','polygon':unique}
