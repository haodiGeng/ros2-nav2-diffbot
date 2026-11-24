#!/usr/bin/env python3
"""Discrete own-robot geometry / world-pose checks for the fixed M4 test box."""
import json
import math
from pathlib import Path
import sys

BOX=[(1.,-.2),(1.4,-.2),(1.4,.2),(1.,.2)]
def rectangle(x,y,yaw,cx,cy,hx,hy):
    c,s=math.cos(yaw),math.sin(yaw)
    return [(x+c*(cx+dx)-s*(cy+dy),y+s*(cx+dx)+c*(cy+dy))
            for dx,dy in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]]
def overlap(a,b):
    for polygon in (a,b):
        for i,p in enumerate(polygon):
            q=polygon[(i+1)%len(polygon)];axis=(-(q[1]-p[1]),q[0]-p[0])
            aa=[x*axis[0]+y*axis[1] for x,y in a]
            bb=[x*axis[0]+y*axis[1] for x,y in b]
            if max(aa)<min(bb) or max(bb)<min(aa):return False
    return True
def point_segment(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
def gap(a,b):
    if overlap(a,b):return 0.
    return min(point_segment(p,poly[i],poly[(i+1)%len(poly)])
               for points,poly in [(a,b),(b,a)] for p in points for i in range(len(poly)))
def robot_gaps(x,y,yaw):
    # Body center -0.04 m; include conservative full wheel projection envelopes.
    parts=[rectangle(x,y,yaw,-.04,0.,.225,.175),
           rectangle(x,y,yaw,0.,.2,.075,.02),rectangle(x,y,yaw,0.,-.2,.075,.02)]
    values=[gap(p,BOX) for p in parts]
    cx=x-.18*math.cos(yaw);cy=y-.18*math.sin(yaw)
    values.append(max(0.,math.hypot(max(abs(cx-1.2)-.2,0.),max(abs(cy)-.2,0.))-.025))
    return values
assert min(robot_gaps(0.,0.,0.))>0.
assert min(robot_gaps(1.2,0.,0.))==0.
for filename in (sys.argv[1:] if __name__=='__main__' else []):
    path=Path(filename);summaries=[]
    for trial in json.loads(path.read_text()):
        truth=trial['truth'];assert truth,'Missing world trajectory'
        distances=[min(robot_gaps(*p[1:])) for p in truth]
        errors=[]
        for pose in trial['amcl']:
            nearest=min(truth,key=lambda p:abs(p[0]-pose[0]))
            if abs(nearest[0]-pose[0])<.2:errors.append(math.hypot(pose[1]-nearest[1],pose[2]-nearest[2]))
        summary={k:trial[k] for k in ('goal','status','wall_seconds','recoveries')}
        summary.update(truth_samples=len(truth),sampled_model_overlaps=sum(d<=0 for d in distances),
            minimum_sampled_box_clearance=min(distances),
            physical_path_length=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(truth,truth[1:])),
            endpoint_world_xy=truth[-1][1:3],
            map_goal_to_world_endpoint_distance=math.hypot(truth[-1][1]-trial['goal'][0],truth[-1][2]-trial['goal'][1]),
            max_amcl_world_position_difference=max(errors) if errors else None,
            paired_localization_samples=len(errors))
        if trial['amcl']:
            last=trial['amcl'][-1]
            summary['last_amcl_goal_distance']=math.hypot(last[1]-trial['goal'][0],last[2]-trial['goal'][1])
            summary['last_amcl_goal_yaw_error']=abs(math.atan2(math.sin(last[3]-trial['goal'][2]),math.cos(last[3]-trial['goal'][2])))
        summaries.append(summary)
    path.with_name(path.stem+'_summary.json').write_text(json.dumps(summaries,indent=2))
    print(path.name,json.dumps(summaries),flush=True)
