#!/usr/bin/env python3
"""Summarize the finite M7 calibrated regression, without ranking algorithms."""
from pathlib import Path
import hashlib,json,math,statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

root=Path.home()/'nav_ws';out=root/'artifacts/m7_calibrated_comparison'
report=json.loads((out/'report.json').read_text());assert report['passed']
boxes=json.loads((root/'artifacts/m7_continuous_v2/geometry.json').read_text())
fig,axes=plt.subplots(2,2,figsize=(10,10),constrained_layout=True)
summary=[]
for run,ax in zip(report['runs'],axes.flat):
 d=json.loads(Path(run['record']).read_text())
 assert abs(d['effective_drive_separation']-.398)<1e-9
 for x,y,sx,sy in boxes:ax.add_patch(Rectangle((x-sx/2,y-sy/2),sx,sy,facecolor='.3'))
 for p in d['phases']:
  points=[q for q in d['truth'] if p['start']<=q[0]<=p['end']]
  ax.plot([q[1] for q in points],[q[2] for q in points],label=p['label'])
 ax.set(xlim=(-4.2,4.2),ylim=(-4.2,4.2),xlabel='World x [m]',ylabel='World y [m]',
        title=f"{run['planner']} / repeat {run['repetition']}")
 ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(fontsize=7)
 p=run['phases']
 summary.append({'planner':run['planner'],'repetition':run['repetition'],
  'successful_goals':sum(x['status']==4 for x in p),'recoveries':sum(x['recoveries'] for x in p),
  'total_sim_seconds':sum(x['sim_seconds'] for x in p),
  'traveled_distance':sum(x['traveled_distance'] for x in p),
  'world_goal_error_max':max(x['world_goal_error'] for x in p),
  'minimum_sampled_circle_clearance':min(x['sampled_circle_clearance'] for x in p),
  'initial_plan_length_sum':sum(x['first_plan_length'] for x in p) if all(x['first_plan_length'] is not None for x in p) else None})
fig.savefig(out/'world_trajectories.png',dpi=160)

regressions=[]
for trial in [1,2]:
 folder=root/f'artifacts/m7_calibrated_regression{trial}'
 d=json.loads((folder/'results.json').read_text());rows=d['rows']
 assert [p['status'] for p in rows]==[4,4,4,4,6,4]
 assert all(p['recoveries']==0 for p in rows)
 assert rows[4]['navigation_goal_sent'] is False
 gaps=[min(math.hypot(max(abs(p[1]-x)-sx/2,0),max(abs(p[2]-y)-sy/2,0))-.35
           for x,y,sx,sy in boxes) for p in d['truth']]
 regressions.append({'trial':trial,'navigation_successes':5,'preflight_refusals':1,'recoveries':0,
                     'minimum_sampled_circle_clearance':min(gaps),'sampled_circle_overlap_count':sum(x<=0 for x in gaps)})
protected=json.loads((root/'artifacts/m7/final_audit.json').read_text())['source_sha256']
keys=[p for p in protected if p.startswith('src/my_planner/') or p in [
 'src/my_robot_navigation/config/nav2_params.yaml','src/my_robot_navigation/config/nav2_dwb_params.yaml',
 'src/my_robot_navigation/config/nav2_rpp_params.yaml','src/my_robot_navigation/maps/my_robot_training.pgm']]
unchanged={p:hashlib.sha256((root/p).read_bytes()).hexdigest()==protected[p] for p in keys}
assert all(unchanged.values())
audit={'comparison_runs':summary,'continuous_regressions':regressions,'protected_sources_unchanged':unchanged,
 'scene_sha256':{p:hashlib.sha256((root/'artifacts/m7_continuous_v2'/p).read_bytes()).hexdigest()
                 for p in ['world.sdf','map.yaml','map.pgm','geometry.json']},
 'limitations':['Two repeats per planner, not statistical significance',
 'Comparison uses zero extra clearance for both planners; production gate tested separately',
 'Sampled conservative circle clearance is not a continuous collision proof',
 'Localization randomness and replanning remain; simulation times are not planner CPU times',
 'No new old-map, dynamic-obstacle or hardware acceptance']}
(out/'acceptance_summary.json').write_text(json.dumps(audit,indent=2))
print(json.dumps(audit,indent=2))
