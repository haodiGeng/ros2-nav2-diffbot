#!/usr/bin/env python3
"""Summarize empirical passage width tests and preserve first-failure snapshots."""
from pathlib import Path
import json,math,sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from summarize_navigation import rectangle,gap

root=Path(sys.argv[1]);rows=[];evidence=[]
fig,ax=plt.subplots(figsize=(10,5),constrained_layout=True)
for folder in sorted(root.iterdir()):
    if not folder.is_dir() or not (folder/'trial.json').exists():continue
    t=json.loads((folder/'trial.json').read_text());meta=json.loads((folder/'geometry.json').read_text())
    width=int(meta['scene'][3:])/100
    obstacles=[[(x0,y0),(x1,y0),(x1,y1),(x0,y1)] for x0,y0,x1,y1 in meta['obstacles']]
    clearances=[]
    for stamp,x,y,yaw in t['truth']:
        parts=[rectangle(x,y,yaw,-.04,0,.225,.175),rectangle(x,y,yaw,0,.2,.075,.02),rectangle(x,y,yaw,0,-.2,.075,.02),rectangle(x,y,yaw,-.18,0,.025,.025)]
        clearances.append(min(gap(a,b) for a in parts for b in obstacles))
    inside=[p for p in t['truth'] if .8<=p[1]<=1.8]
    row={k:t.get(k) for k in ['status','recoveries','wall_seconds','navigation_goal_sent','preflight_status','path_points',
        'max_translation_during_rejection','nonzero_commands','runtime_aborted','max_translation_after_abort_settled','nonzero_commands_after_abort_settled']}
    row.update(case=folder.name,width=width,mission_succeeded=t['status']==4,minimum_sampled_body_clearance=min(clearances),
        sampled_overlaps=sum(x==0 for x in clearances),max_abs_world_y_in_passage=max((abs(p[2]) for p in inside),default=None))
    row['stable_traversal']=t['status']==4 and t['recoveries']==0 and row['sampled_overlaps']==0
    rows.append(row)
    xy=t['truth'];ax.plot([p[1] for p in xy],[p[2] for p in xy],label=folder.name,alpha=.7)
    for snapshot in t.get('failure_costmaps',[])[:1]:
        if 'cell_xy' not in snapshot:continue
        truth=snapshot['truth'];estimated=snapshot['map_base_xy']
        plans=[p for p in t.get('plans',[]) if p['stamp']<=snapshot['event']['stamp']]
        points=plans[-1]['points'] if plans else []
        near=[p for p in points if abs(p[0]-truth[1])<.15]
        item={'case':folder.name,'event':snapshot['event'],'sample_stamp':snapshot['sample_stamp'],'truth':truth,
              'map_base_xy':estimated,'estimate_minus_world_y':estimated[1]-truth[2],
              'blocked_cell':snapshot['cell_xy'],'blocked_cost':snapshot['cell_cost'],
              'nearby_planned_y':sum(p[1] for p in near)/len(near) if near else None}
        evidence.append(item)
ax.axhline(0,color='black',lw=.8);ax.set(xlabel='World x [m]',ylabel='World y [m]',title='Passage tracking: recorded lateral displacement')
ax.legend(fontsize=7,ncol=2);ax.grid(alpha=.2);fig.savefig(root/'lateral_paths.png',dpi=160)
(root/'summary.json').write_text(json.dumps(rows,indent=2));(root/'failure_evidence.json').write_text(json.dumps(evidence,indent=2))
lines=['| Case | Width m | Status | Recoveries | Min body gap m | Stable traversal |','|---|---:|---:|---:|---:|---|']
for r in rows:lines.append(f"| {r['case']} | {r['width']:.2f} | {r['status']} | {r['recoveries']} | {r['minimum_sampled_body_clearance']:.3f} | {r['stable_traversal']} |")
(root/'measurements.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'rows':rows,'first_failures':evidence},indent=2))
