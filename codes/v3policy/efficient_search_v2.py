"""Q3 travel optimization using public bounded-error observations only."""
import math
from .geometry import minimum_enclosing_circle
from .problem2_model import candidate_points, robust_score
from .efficient_search import EfficientPolicy, coverage_points, next_cover_point


def target_route(current, points):
    """Nearest-neighbour open route improved by endpoint-inclusive 2-opt."""
    remaining=list(points)
    route=[]
    last=current
    while remaining:
        q=min(remaining,key=lambda q:math.dist(last,q))
        remaining.remove(q);route.append(q);last=q
    def length(path):
        return math.dist(current,path[0])+sum(math.dist(a,b) for a,b in zip(path,path[1:])) if path else 0
    best=length(route)
    for _ in range(4):
        improved=False
        for i in range(len(route)):
            for j in range(i+1,len(route)):
                trial=route[:i]+route[i:j+1][::-1]+route[j+1:]
                cost=length(trial)
                if cost<best-1e-7:
                    route,best=trial,cost;improved=True
        for i in range(len(route)):
            for j in range(len(route)):
                trial=list(route);point=trial.pop(i);trial.insert(j,point)
                cost=length(trial)
                if cost<best-1e-7:
                    route,best=trial,cost;improved=True
        if not improved:break
    return route


def measurement_point(poly,current,excluded=(),weight=.5,bin_width=4):
    center,_=minimum_enclosing_circle(poly)
    distance=math.dist(current,center)
    ux,uy=((center[0]-current[0])/distance,(center[1]-current[1])/distance) if distance else (1.,0.)
    candidates=candidate_points(poly,current)
    for fraction in (0.,.25,.5,.75,1.):
        for lateral in (0.,-20.,20.,-40.,40.,-80.,80.,-140.,140.,-200.,200.):
            candidates.append((current[0]+fraction*distance*ux-lateral*uy,
                               current[1]+fraction*distance*uy+lateral*ux))
    best=math.inf;chosen=None;seen=set()
    for q in candidates:
        key=tuple(round(v,6) for v in q)
        if key in seen or any(math.dist(q,p)<.01 for p in [current,*excluded]):continue
        seen.add(key)
        if max(math.dist(q,p) for p in poly)>999.9:continue
        travel=math.dist(current,q)+math.dist(q,center)
        # Early rejection preserves the selected score; no impossible angular
        # bins are dropped and no uncertainty bounds are narrowed.
        cutoff=(best-travel)/weight
        if cutoff<0:continue
        diameter=robust_score(poly,q,bin_width=bin_width,cutoff=cutoff)
        score=travel+weight*diameter
        if score<best:best,chosen=score,q
    if chosen is None:raise ValueError('no range-guaranteed candidate')
    return chosen


class FasterPolicy(EfficientPolicy):
    uncertainty_weight=.25
    # 2° 分箱仍把每个真实示向角包含在“测量误差 + 分箱半宽”的楔形内；
    # 24 个本地同构 Engine 种子复核均保持审计通过，并较4°少走约86 m/例。
    angle_bin=2
    route_targets=True
    plan_cover_route=False

    def localize(self,channel,first,first_position=None):
        result=first
        pos=self.position if first_position is None else first_position
        measured=[pos]
        for iteration in range(6):
            if result['measure_result']=='near':
                if self.clear(pos,channel):return
                raise RuntimeError('near response contradicted by clear')
            poly=self.update(channel,pos,result)
            center,radius=minimum_enclosing_circle(poly)
            self.trace.append(dict(channel=channel,iteration=iteration,radius_m=radius))
            if radius<=19.99:
                if self.clear(center,channel):return
                raise RuntimeError('covering-circle guarantee contradicted')
            if radius<90 and self.clear(center,channel):return
            try:q=measurement_point(poly,self.position,measured,self.uncertainty_weight,self.angle_bin)
            except ValueError:break
            result=self.measure(q,channel);pos=q;measured.append(q)
        if result['measure_result']=='near':
            if self.clear(pos,channel):return
            raise RuntimeError('near response contradicted by clear')
        self.optical_cover(channel,self.update(channel,pos,result))

    def run(self):
        if not self.route_targets:return super().run()
        if self.client.enter().get('accepted') is not True:raise RuntimeError('entry refused')
        pending=coverage_points(ring_radius=self.ring_radius)
        while pending:
            q=next_cover_point(self.position,pending) if self.plan_cover_route else min(pending,key=lambda p:math.dist(self.position,p))
            pending.remove(q)
            unresolved=[c for c in range(1,21) if c not in self.cleared]
            if self.channel in unresolved:
                unresolved.remove(self.channel);unresolved.insert(0,self.channel)
            detected=[]
            for channel in unresolved:
                result=self.measure(q,channel)
                if result['measure_result']!='no_signal':
                    self.update(channel,q,result);detected.append((channel,result,q))
            while detected:
                centers=[minimum_enclosing_circle(self.regions[e[0]])[0] if e[0] in self.regions else e[2] for e in detected]
                first=target_route(self.position,centers)[0]
                entry=detected.pop(centers.index(first))
                self.localize(*entry)
                if self.share_observations:
                    for index,(channel,previous,previous_pos) in enumerate(detected):
                        result=self.measure(self.position,channel)
                        if result['measure_result']!='no_signal':
                            self.update(channel,self.position,result)
                            detected[index]=(channel,result,self.position)
            if len(self.cleared)==16:
                self.stop_reason='known_upper_bound_reached';break
        if self.stop_reason is None:self.stop_reason='all_unresolved_channels_covered'
        self.client.exit()
        return dict(cleared=len(self.cleared),stop_reason=self.stop_reason,fallback_count=self.fallback_count)
