#!/usr/bin/env python3
"""Recompute M8 tables from archived evidence. Never starts or drives ROS nodes."""
import argparse,csv,hashlib,json,statistics,xml.etree.ElementTree as ET
from pathlib import Path

def read(path):
 return json.loads(path.read_text(encoding='utf-8-sig'))

def write_csv(path,rows):
 with path.open('w',newline='',encoding='utf-8-sig') as stream:
  writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)

def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
 parser.add_argument('--no-plots',action='store_true')
 args=parser.parse_args();root=args.root.resolve();out=root/'artifacts/m8/evaluation';out.mkdir(parents=True,exist_ok=True)
 inputs=['artifacts/m7/planner_comparison.json','artifacts/m7/frozen_costmap.json',
         'artifacts/m7/test_astar_core.gtest.xml','artifacts/m7_calibrated_comparison/report.json',
         'artifacts/m7_calibrated_comparison/acceptance_summary.json',
         'artifacts/m7_calibrated_comparison/desktop_trial.json',
         'artifacts/m7_calibrated_regression1/results.json','artifacts/m7_calibrated_regression2/results.json']
 p=read(root/inputs[0]);assert p['passed'] and p['costmap_unchanged']
 frozen=read(root/inputs[1]);assert hashlib.sha256(bytes(frozen['data'])).hexdigest()==p['costmap_sha256']
 measured=p['measured_rows'];assert len(measured)==105
 fixed=[]
 for pair in range(3):
  for planner in p['variants']:
   rows=[r for r in measured if r['pair']==pair and r['planner']==planner]
   assert len(rows)==5 and all(r['status']==4 for r in rows)
   original=next(s for s in p['summaries'] if s['pair']==pair and s['planner']==planner)
   ms=statistics.median(r['planning_seconds']*1000 for r in rows)
   assert abs(ms-original['median_planning_ms'])<1e-6
   fixed.append(dict(query=pair,planner=planner,measured_requests=len(rows),
    median_planning_ms=ms,median_path_length_m=statistics.median(r['length'] for r in rows),
    sampled_center_obstacle_distance_m=original['sampled_min_center_obstacle_distance'],
    sampled_costmap_safe=original['sampled_base_costmap_safe']))
 assert all(r['sampled_costmap_safe'] for r in fixed)
 assert len(p['rejected_goals'])==2 and all(r['status']==6 and r['pose_count']==0 for r in p['rejected_goals'])
 tests=ET.parse(root/inputs[2]).getroot();cases=tests.findall('.//testcase')
 assert len(cases)==14 and not tests.findall('.//failure') and not tests.findall('.//error')
 comparison=read(root/inputs[3]);assert comparison['passed'] and len(comparison['runs'])==4
 closed=[]
 for run in comparison['runs']:
  phases=run['phases'];assert len(phases)==4
  assert all(x['status']==4 and x['recoveries']==0 for x in phases)
  closed.append(dict(planner=run['planner'],repetition=run['repetition'],successful_goals=4,attempted_goals=4,
   recoveries=sum(x['recoveries'] for x in phases),sim_seconds=sum(x['sim_seconds'] for x in phases),
   actual_distance_m=sum(x['traveled_distance'] for x in phases),
   max_world_goal_error_m=max(x['world_goal_error'] for x in phases),
   minimum_sampled_circle_clearance_m=min(x['sampled_circle_clearance'] for x in phases)))
 assert sorted((r['planner'],r['repetition']) for r in closed)==[('astar',1),('astar',2),('navfn',1),('navfn',2)]
 gates=[]
 for trial in [1,2]:
  d=read(root/f'artifacts/m7_calibrated_regression{trial}/results.json');rows=d['rows']
  assert [r['status'] for r in rows]==[4,4,4,4,6,4]
  assert not rows[4]['navigation_goal_sent'] and all(r['recoveries']==0 for r in rows)
  gates.append(dict(trial=trial,navigation_goals=5,navigation_successes=5,
   expected_preflight_refusals=1,actual_preflight_refusals=1,recoveries=0))
 desktop=read(root/inputs[5]);assert desktop['completed'] and len(desktop['rows'])==2
 assert all(r['status']==4 and r['recoveries']==0 for r in desktop['rows'])
 audit=read(root/inputs[4]);assert all(audit['protected_sources_unchanged'].values())
 report={'scope':'Archived M7 static-scene evidence, consolidated for M8; no new navigation trial',
  'core_tests_passed':len(cases),'fixed_map_measured_requests':len(measured),
  'fixed_map_warmup_policy':'One excluded warm-up per query and variant; five measured requests',
  'fixed_map_sha256':p['costmap_sha256'],'rejected_invalid_goals':2,
  'fixed_map':fixed,'closed_loop':closed,'gated_missions':gates,'desktop_goals_succeeded':2,
  'evidence_sha256':{name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in inputs},
  'limitations':['Two fresh simulations per planner; goals within a simulation are correlated',
   'Planner-server planning duration differs from action round-trip and navigation time',
   'Fixed-map center distance differs from actual robot circle clearance',
   'Goal-to-world endpoint error includes localization and navigation tolerance',
   'No statistical superiority, dynamic-obstacle, precision-docking or hardware claim'],
  'passed':True}
 write_csv(out/'fixed_map.csv',fixed);write_csv(out/'closed_loop.csv',closed);write_csv(out/'gated_missions.csv',gates)
 (out/'evaluation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
 lines=['# M8 unified evaluation','',report['scope'],'',
  '## Closed-loop navigation','',
  '| Planner / repeat | Success | Recovery | Sim time / s | Actual distance / m | Max world goal error / m | Min sampled circle gap / m |',
  '|---|---:|---:|---:|---:|---:|---:|']
 for r in closed:
  lines.append(f"| {r['planner']} / {r['repetition']} | 4/4 | 0 | {r['sim_seconds']:.2f} | {r['actual_distance_m']:.2f} | {r['max_world_goal_error_m']:.3f} | {r['minimum_sampled_circle_clearance_m']:.3f} |")
 lines+=['','## Fixed-map planning','',
  '| Query | Planner | Requests | Median planning / ms | Median polyline / m | Sampled center gap / m |',
  '|---|---|---:|---:|---:|---:|']
 for r in fixed:lines.append(f"| {r['query']} | {r['planner']} | 5 | {r['median_planning_ms']:.3f} | {r['median_path_length_m']:.3f} | {r['sampled_center_obstacle_distance_m']:.3f} |")
 lines+=['','## Other acceptance','',
  '14 core tests; 105 measured planning requests; 2 invalid goals rejected; 10 gated navigation successes and 2 expected preflight refusals; 2 GUI navigation successes.',
  '', '## Limits','']+['- '+x for x in report['limitations']]
 (out/'comparison.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
 if not args.no_plots:
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  fig,ax=plt.subplots(figsize=(12,5),constrained_layout=True)
  width=.11
  for i,name in enumerate(p['variants']):
   vals=[r['median_planning_ms'] for r in fixed if r['planner']==name]
   ax.bar([q+(i-3)*width for q in range(3)],vals,width=width,label=name)
  ax.set(xticks=[0,1,2],xticklabels=['Query 0','Query 1','Query 2'],ylabel='Median planner-server time [ms]',title='Fixed-map planning: 5 measured requests after warm-up')
  ax.legend(fontsize=8,ncol=3);ax.grid(axis='y',alpha=.2);fig.savefig(out/'fixed_map_planning.png',dpi=160);plt.close(fig)
  fig,axes=plt.subplots(1,3,figsize=(13,4),constrained_layout=True)
  labels=[f"{r['planner']} {r['repetition']}" for r in closed]
  for ax,key,title in zip(axes,['sim_seconds','actual_distance_m','max_world_goal_error_m'],['Mission simulation time [s]','Actual traveled distance [m]','Maximum world goal error [m]']):
   ax.bar(labels,[r[key] for r in closed],color=['#3978b7' if r['planner']=='astar' else '#de8f36' for r in closed]);ax.set_title(title);ax.tick_params(axis='x',rotation=25);ax.grid(axis='y',alpha=.2)
  fig.savefig(out/'closed_loop_metrics.png',dpi=160);plt.close(fig)
 print(json.dumps({'passed':True,'core_tests':len(cases),'planning_requests':len(measured),'closed_loop_goals':16,'output':str(out)}))

if __name__=='__main__':main()
