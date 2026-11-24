#!/usr/bin/env python3
from pathlib import Path
import json,math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path.home()/'nav_ws/artifacts';out=r/'m7_reverse_summary';out.mkdir(exist_ok=True)
reports=[];fig,axes=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
for name in ['m7_reverse_debug','m7_reverse_release1','m7_reverse_release2']:
 d=json.loads((r/name/'results.json').read_text());boxes=json.loads((r/name/'geometry.json').read_text())
 row=next(x for x in d['rows'] if x['label']=='return_again')
 poses=[p for p in d['truth'] if row['sim_start']<=p[0]<=row['sim_end']]
 axes[0].plot([p[0]-row['sim_start'] for p in poses],[p[1] for p in poses],label=name)
 clearance=min(math.hypot(max(abs(p[1]-x)-sx/2,0),max(abs(p[2]-y)-sy/2,0))-.35 for p in d['truth'] for x,y,sx,sy in boxes)
 reports.append({'run':name,'rows':d['rows'],'sampled_circle_clearance':clearance,
   'circle_overlap_samples':sum(any(math.hypot(max(abs(p[1]-x)-sx/2,0),max(abs(p[2]-y)-sy/2,0))<=.35 for x,y,sx,sy in boxes) for p in d['truth'])})
 if name=='m7_reverse_release1':
  cmd=[x for x in d.get('commands',[]) if row['sim_start']<=x[0]<=row['sim_end']]
  axes[1].plot([x[0]-row['sim_start'] for x in cmd],[x[1] for x in cmd],label='forward vx [m/s]')
  axes[1].plot([x[0]-row['sim_start'] for x in cmd],[x[2] for x in cmd],label='angular w [rad/s]')
 if name!='m7_reverse_debug':
  assert [x['status'] for x in d['rows']]==[4,4,4,4,6,4]
  assert all(x['recoveries']==0 for x in d['rows'])
  assert not d['rows'][4]['navigation_goal_sent']
  assert clearance>0
  assert max(abs(x[1]) for x in d['commands'])<=.180001
  assert max(abs(x[2]) for x in d['commands'])<=.600001
axes[0].set(title='Reverse return: world x',xlabel='Simulation time after request [s]',ylabel='World x [m]')
axes[1].set(title='Release run: align heading, then track',xlabel='Simulation time after request [s]',ylabel='Command value (units in legend)')
for ax in axes:ax.legend(fontsize=7);ax.grid(alpha=.2)
fig.savefig(out/'reverse_comparison.png',dpi=160)
(out/'report.json').write_text(json.dumps(reports,indent=2))
print(json.dumps([{k:v for k,v in row.items() if k!='rows'} for row in reports],indent=2))
