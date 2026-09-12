"""Q3 scan relocation with conservative continuous-area coverage certificates."""
import math
from functools import lru_cache
from .geometry import initial_region,clip_halfplane,minimum_enclosing_circle
from .efficient_search import coverage_points
from .efficient_search_v2 import FasterPolicy,target_route


def convex_hull(points):
    points=sorted(set(points))
    if len(points)<3:return points
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower=[];upper=[]
    for p in points:
        while len(lower)>1 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper)>1 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


@lru_cache(maxsize=1)
def partition():
    anchors=tuple(coverage_points())
    arena=initial_region()
    cells=[];owners=[]
    for x in range(-1800,1801,40):
        for y in range(-1800,1801,40):
            if max(abs(x)-20,0)**2+max(abs(y)-20,0)**2>1801**2:continue
            poly=arena
            for a,b,c in ((1,0,x+20),(-1,0,-x+20),(0,1,y+20),(0,-1,-y+20)):
                poly=clip_halfplane(poly,a,b,c)
                if not poly:break
            if not poly:continue
            radius,owner=min((max(math.dist(q,p) for p in poly),i) for i,q in enumerate(anchors))
            if radius>999.99:raise RuntimeError('Uncovered partition cell')
            cells.append(tuple(poly));owners.append(owner)
    return anchors,tuple(cells),tuple(owners)


class AdaptiveCoverage:
    radius=999.98
    def __init__(self):
        self.anchors,self.cells,self.owners=partition()
        self.remaining=set(range(len(self.cells)))
        self.hulls={}
        self.refresh()
    def refresh(self):
        self.hulls={owner:convex_hull(p for i in sorted(self.remaining) if self.owners[i]==owner for p in self.cells[i]) for owner in range(7)}
    def record(self,pos,owner=None):
        # Convex disk contains a whole cell iff it contains every vertex.
        self.remaining={i for i in self.remaining if any(math.dist(pos,p)>999.99 for p in self.cells[i])}
        self.refresh()
    def choices(self,current):
        choices=[]
        for owner,poly in self.hulls.items():
            if not poly:continue
            anchor=self.anchors[owner]
            def fits(q):return all(math.dist(q,p)<=self.radius for p in poly)
            if fits(current):q=current
            else:
                lo,hi=0.,1.
                for _ in range(32):
                    fraction=(lo+hi)/2
                    q=(anchor[0]+fraction*(current[0]-anchor[0]),anchor[1]+fraction*(current[1]-anchor[1]))
                    if fits(q):lo=fraction
                    else:hi=fraction
                q=(anchor[0]+lo*(current[0]-anchor[0]),anchor[1]+lo*(current[1]-anchor[1]))
            if not fits(q):raise RuntimeError('Scan relocation lost coverage guarantee')
            choices.append((owner,q))
        return choices


class FlexibleCoverage(AdaptiveCoverage):
    """A relocated scan handles cells no other remaining anchor can cover.

    Shared cells may be left for a future anchor only when its entire cell is
    inside that anchor's guaranteed reception disk. This maintains a feasible
    complete cover after each anchor is consumed; uncovered cells are not lost.
    """
    def __init__(self):
        super().__init__()
        self.pending=set(range(7))
        self.fixed=[{i for i,cell in enumerate(self.cells) if all(math.dist(q,p)<=999.99 for p in cell)} for q in self.anchors]
        if set.union(*self.fixed)!=self.remaining:raise RuntimeError('Initial full cover is not certified')
    def critical(self,owner):
        others=set().union(*(self.fixed[j] for j in self.pending if j!=owner))
        return self.remaining-others
    def choices(self,current):
        for owner in sorted(self.pending):
            if not self.critical(owner):self.pending.remove(owner)
        self.hulls={owner:convex_hull(p for i in sorted(self.critical(owner)) for p in self.cells[i]) for owner in sorted(self.pending)}
        result=super().choices(current)
        if self.remaining and not result:raise RuntimeError('Remaining region has no certified future scan')
        return result
    def record(self,pos,owner=None):
        if owner not in self.pending:raise RuntimeError('Missing planned scan owner')
        critical=self.critical(owner)
        super().record(pos)
        if critical & self.remaining:raise RuntimeError('Critical region not fully scanned')
        self.pending.remove(owner)
        self.hulls={j:convex_hull(p for i in sorted(self.critical(j)) for p in self.cells[i]) for j in sorted(self.pending)}


def route_with_exit(current,points,exits):
    route=target_route(current,points)
    if not exits or not route:return route
    def length(r):return math.dist(current,r[0])+sum(math.dist(a,b) for a,b in zip(r,r[1:]))+min(math.dist(r[-1],q) for q in exits)
    best=length(route)
    for _ in range(4):
        improved=False
        for i in range(len(route)):
            for j in range(i+1,len(route)):
                trial=route[:i]+route[i:j+1][::-1]+route[j+1:]
                score=length(trial)
                if score<best-1e-7:route,best,improved=trial,score,True
        for i in range(len(route)):
            for j in range(len(route)):
                trial=list(route);q=trial.pop(i);trial.insert(j,q)
                score=length(trial)
                if score<best-1e-7:route,best,improved=trial,score,True
        if not improved:break
    return route


class AdaptivePolicy(FasterPolicy):
    dynamic_scans=True
    joint_route=True
    coverage_class=FlexibleCoverage
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        if self.directional:raise ValueError('Q3 scan coverage requires omnidirectional sources')
        self.coverage=self.coverage_class() if self.dynamic_scans else None
        self.scan_positions=[]

    def run(self):
        if self.client.enter().get('accepted') is not True:raise RuntimeError('entry refused')
        pending=list(coverage_points())
        while True:
            if self.dynamic_scans:
                choices=self.coverage.choices(self.position)
                if not choices:break
                owner,q=min(choices,key=lambda pair:math.dist(self.position,pair[1]))
            else:
                if not pending:break
                q=min(pending,key=lambda p:math.dist(self.position,p));pending.remove(q)
            self.scan_positions.append(q)
            unresolved=[c for c in range(1,21) if c not in self.cleared]
            if self.channel in unresolved:
                unresolved.remove(self.channel);unresolved.insert(0,self.channel)
            detected=[]
            for channel in unresolved:
                result=self.measure(q,channel)
                if result['measure_result']!='no_signal':
                    self.update(channel,q,result);detected.append((channel,result,q))
            if self.dynamic_scans:
                previous=len(self.coverage.remaining)
                self.coverage.record(q,owner)
                if len(self.coverage.remaining)>=previous:raise RuntimeError('No certified search progress')
                exits=[self.coverage.anchors[i] for i,p in self.coverage.hulls.items() if p]
            else:exits=pending
            # Every positive observation is cleared before consulting the
            # coverage certificate to decide that the search is complete.
            while detected:
                centers=[minimum_enclosing_circle(self.regions[e[0]])[0] if e[0] in self.regions else e[2] for e in detected]
                route=route_with_exit(self.position,centers,exits) if self.joint_route else target_route(self.position,centers)
                entry=detected.pop(centers.index(route[0]));self.localize(*entry)
                if self.share_observations:
                    for index,(channel,previous,previous_pos) in enumerate(detected):
                        result=self.measure(self.position,channel)
                        if result['measure_result']!='no_signal':
                            self.update(channel,self.position,result);detected[index]=(channel,result,self.position)
            if len(self.cleared)==16:
                self.stop_reason='known_upper_bound_reached';break
        if self.stop_reason is None:self.stop_reason='all_unresolved_channels_covered'
        self.client.exit()
        return dict(cleared=len(self.cleared),stop_reason=self.stop_reason,fallback_count=self.fallback_count,full_scan_positions=len(self.scan_positions))
