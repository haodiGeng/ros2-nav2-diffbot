#!/usr/bin/env python3
"""Paired navigation experiments; normal costmap safety, no extra-clearance gate.

The existing no-recovery tree names its planner ClearanceGate. Only in these
experimental YAML files that alias is assigned to the planner under comparison.
This does not validate the production gate's additional clearance policy.
"""
from pathlib import Path
import hashlib,json,math,os,subprocess,sys
import yaml

root=Path.home()/'nav_ws'
out=root/'artifacts/m7_calibrated_comparison';out.mkdir(parents=True,exist_ok=True)
source=root/'src/my_robot_navigation/config/nav2_amcl_sim_calibrated.yaml'
base=yaml.safe_load(source.read_text())
variants={}
for name,plugin in [('astar','GridBased'),('navfn','NavFnBaseline')]:
 cfg=yaml.safe_load(source.read_text())
 p=cfg['planner_server']['ros__parameters']
 p['ClearanceGate']=dict(p[plugin])
 variants[name]=cfg
 (out/(name+'.yaml')).write_text(yaml.safe_dump(cfg))

def normalized(cfg):
 c=yaml.safe_load(yaml.safe_dump(cfg));del c['planner_server']['ros__parameters']['ClearanceGate']
 return c
assert normalized(variants['astar'])==normalized(variants['navfn'])
manifest={'passed':False,'track_scale':.995,'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
 'only_parameter_difference':'planner_server.ClearanceGate plugin and plugin-specific parameters',
 'extra_clearance_policy':False,'order':['astar1','navfn1','navfn2','astar2'],'runs':[]}
(out/'report.json').write_text(json.dumps(manifest,indent=2))
script=root/'src/my_robot_navigation/scripts/localization_study.py'
geometry=json.loads((root/'artifacts/m7_continuous_v2/geometry.json').read_text())
for name,repetition in [('astar',1),('navfn',1),('navfn',2),('astar',2)]:
 tag=f'compare_{name}{repetition}';record=root/'artifacts/m7_localization'/f'{tag}_loop/record.json'
 if record.exists():raise RuntimeError('Refusing to overwrite existing experiment '+str(record))
 print('Starting '+tag,flush=True)
 with (out/(tag+'.log')).open('w') as log:
  result=subprocess.run([sys.executable,str(script),'loop',tag,str(out/(name+'.yaml')),'0.995'],stdout=log,stderr=log)
 if result.returncode:raise RuntimeError(f'{tag} failed, see its log')
 d=json.loads(record.read_text());assert d.get('completed') and len(d['phases'])==4
 phases=[]
 for phase in d['phases']:
  assert phase['status']==4 and phase['recoveries']==0
  truth=[p for p in d['truth'] if phase['start']<=p[0]<=phase['end']]
  assert truth
  gap=min(min(math.hypot(max(abs(p[1]-x)-sx/2,0),max(abs(p[2]-y)-sy/2,0))-.35
                  for x,y,sx,sy in geometry) for p in truth)
  plans=[p for p in d['plans'] if phase['start']<=p['stamp']<=phase['end'] and p['points']]
  phases.append({'label':phase['label'],'status':phase['status'],'recoveries':phase['recoveries'],
   'sim_seconds':phase['end']-phase['start'],
   'traveled_distance':sum(math.dist(a[1:3],b[1:3]) for a,b in zip(truth,truth[1:])),
   'world_goal_error':math.dist(truth[-1][1:3],phase['goal']),
   'sampled_circle_clearance':gap,'plan_count':len(plans),
   'first_plan_length':sum(math.dist(a,b) for a,b in zip(plans[0]['points'],plans[0]['points'][1:])) if plans else None})
 manifest['runs'].append({'planner':name,'repetition':repetition,'record':str(record),'phases':phases})
 (out/'report.json').write_text(json.dumps(manifest,indent=2))
 print('Completed '+tag,flush=True)
manifest['passed']=True
(out/'report.json').write_text(json.dumps(manifest,indent=2))
print('All paired navigation experiments completed',flush=True)
