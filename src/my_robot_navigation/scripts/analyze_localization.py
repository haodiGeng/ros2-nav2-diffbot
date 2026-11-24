#!/usr/bin/env python3
from pathlib import Path
import json,math,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(sys.argv[1]) if len(sys.argv)>1 else Path.home()/'nav_ws/artifacts/m7_localization'
def angle(x):return math.atan2(math.sin(x),math.cos(x))
rows=[];fig,axes=plt.subplots(1,3,figsize=(15,4),constrained_layout=True)
for file in sorted(root.glob('*/record.json')):
 d=json.loads(file.read_text());tf=d['synchronized_tf'];assert tf
 initial=tf[0];a=initial['truth'];b=initial['odom_base'];rot=a[2]-b[2];c,s=math.cos(rot),math.sin(rot)
 offset=[a[0]-c*b[0]+s*b[1],a[1]-s*b[0]-c*b[1]]
 estimates=[];odoms=[];yaws=[];odom_yaws=[]
 for p in tf:
  w=p['truth'];m=p['map_base'];o=p['odom_base'];pred=[offset[0]+c*o[0]-s*o[1],offset[1]+s*o[0]+c*o[1]]
  estimates.append(math.hypot(m[0]-w[0],m[1]-w[1]));odoms.append(math.dist(pred,w[:2]));yaws.append(abs(angle(m[2]-w[2])))
  odom_yaws.append(abs(angle(o[2]+rot-w[2])))
 raw=[];raw_yaws=[]
 for p in d['amcl']:
  w=min(d['truth'],key=lambda w:abs(w[0]-p[0]))
  if abs(w[0]-p[0])<.02:
   raw.append(math.dist(p[1:3],w[1:3]));raw_yaws.append(abs(angle(p[3]-w[3])))
 phases=[]
 for phase in d['phases']:
  p=min(tf,key=lambda p:abs(p['stamp']-phase['end']));m=p['map_base'];w=p['truth']
  item={**phase,'sample_stamp':p['stamp'],'truth':w,'map_base':m,'estimate_position_error':math.dist(m[:2],w[:2]),
        'estimate_yaw_error':angle(m[2]-w[2])}
  if 'goal' in phase:
   g=phase['goal'];item.update(world_goal_error=math.dist(w[:2],g),estimated_goal_error=math.dist(m[:2],g),
     world_minus_estimate=[w[0]-m[0],w[1]-m[1]],estimate_minus_goal=[m[0]-g[0],m[1]-g[1]])
  if d['mode']=='rotation':
   part=[x for x in tf if phase['start']<=x['stamp']<=phase['end']]
   if len(part)>1:
    for key in ['truth','odom_base','map_base']:
     values=np.unwrap([x[key][2] for x in part]);item[key+'_yaw_travel']=float(values[-1]-values[0])
  phases.append(item)
 row={'tag':d['tag'],'mode':d['mode'],'completed':d.get('completed',False),'paired_tf_count':len(tf),'tf_failures':d['tf_failures'],
  'tf_map_position_max':max(estimates),'tf_map_position_p95':float(np.percentile(estimates,95)),
  'aligned_odom_position_max':max(odoms),'tf_map_yaw_max':max(yaws),'raw_amcl_position_max':max(raw) if raw else None,'phases':phases}
 row.update(aligned_odom_yaw_max=max(odom_yaws),raw_amcl_yaw_max=max(raw_yaws) if raw_yaws else None)
 tf_stamps={p['stamp'] for p in tf}
 row['missing_tf_samples_during_phases']=sum(p[0] not in tf_stamps for p in d['truth'] if any(q['start']<=p[0]<=q['end'] for q in d['phases']))
 residuals=[]
 for p in tf:
  a=p['map_odom'];b=p['odom_base'];m=p['map_base'];c0,s0=math.cos(a[2]),math.sin(a[2])
  residuals.append(math.hypot(a[0]+c0*b[0]-s0*b[1]-m[0],a[1]+s0*b[0]+c0*b[1]-m[1]))
 row['tf_chain_composition_max_position_residual']=max(residuals)
 geometry=root.parent/'m7_continuous_v2/geometry.json'
 if geometry.exists():
  boxes=json.loads(geometry.read_text())
  gaps=[min(math.hypot(max(abs(p[1]-x)-sx/2,0),max(abs(p[2]-y)-sy/2,0))-.35 for x,y,sx,sy in boxes) for p in d['truth']]
  row['minimum_sampled_circle_clearance']=min(gaps);row['sampled_circle_overlap_count']=sum(x<=0 for x in gaps)
 rows.append(row)
 ax=axes[['straight','rotation','loop'].index(d['mode'])];x=[p['stamp']-tf[0]['stamp'] for p in tf]
 ax.plot(x,estimates,label=d['tag']+' map TF');ax.plot(x,odoms,'--',label=d['tag']+' aligned odom')
 ax.set(title=d['mode'],xlabel='Simulation time [s]',ylabel='Position difference [m]');ax.grid(alpha=.2);ax.legend(fontsize=7)
(root/'summary.json').write_text(json.dumps(rows,indent=2));fig.savefig(root/'position_errors.png',dpi=160)
print(json.dumps(rows,indent=2))
