#!/usr/bin/env python3
"""Sampled footprint clearance and synchronized localization diagnostics."""
import json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from summarize_navigation import rectangle,gap

root=Path(sys.argv[1]) if len(sys.argv)>1 else Path.home()/'nav_ws/artifacts/m7_physical'
rows=[]
fig,axes=plt.subplots(1,2,figsize=(12,5),constrained_layout=True)
for folder in sorted(root.iterdir()):
    if not folder.is_dir() or not (folder/'trial.json').exists():continue
    t=json.loads((folder/'trial.json').read_text());meta=json.loads((folder/'geometry.json').read_text())
    obstacles=[[(x0,y0),(x1,y0),(x1,y1),(x0,y1)] for x0,y0,x1,y1 in meta['obstacles']]
    values=[]
    # Conservative wheel envelopes and caster square contain their true projection.
    for stamp,x,y,yaw in t['truth']:
        parts=[rectangle(x,y,yaw,-.04,0,.225,.175),rectangle(x,y,yaw,0,.2,.075,.02),
               rectangle(x,y,yaw,0,-.2,.075,.02),rectangle(x,y,yaw,-.18,0,.025,.025)]
        values.append(min(gap(a,b) for a in parts for b in obstacles))
    errors=[]
    for p in t['amcl']:
        q=min(t['truth'],key=lambda q:abs(q[0]-p[0]))
        if abs(q[0]-p[0])<=.15:errors.append([p[0],math.hypot(p[1]-q[1],p[2]-q[2]),abs(math.atan2(math.sin(p[3]-q[3]),math.cos(p[3]-q[3])))])
    row={k:t.get(k) for k in ['status','recoveries','wall_seconds','world_endpoint_goal_difference','max_translation_during_rejection','nonzero_commands']}
    row.update(case=folder.name,passed=t['passed'],scene=meta['scene'],planner=meta['planner'],
        minimum_sampled_footprint_clearance=min(values),sampled_overlaps=sum(v==0 for v in values),
        paired_pose_count=len(errors),max_paired_amcl_world_error=max((p[1] for p in errors),default=None),
        max_paired_amcl_world_yaw_error=max((p[2] for p in errors),default=None),
        final_paired_amcl_world_error=errors[-1][1] if errors else None,
        physical_length=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(t['truth'],t['truth'][1:])))
    rows.append(row)
    row['sampled_geometry_passed']=row['sampled_overlaps']==0
    row['stable_following_passed']=None if t['reject_test'] else (t['status']==4 and t['recoveries']==0 and row['sampled_geometry_passed'])
    if meta['scene'] in ['wide','box']:
        ax=axes[0 if meta['scene']=='wide' else 1];xy=np.array(t['truth'])
        ax.plot(xy[:,1],xy[:,2],label=folder.name,alpha=.75)
        if meta['planner']=='astar' and meta['repetition']==0:
            for x0,y0,x1,y1 in meta['obstacles']:ax.add_patch(plt.Rectangle((x0,y0),x1-x0,y1-y0,color='black'))
for ax,title in zip(axes,['Wide passage: recorded world trajectories','Box avoidance: recorded world trajectories']):
    ax.set(title=title,xlabel='world x [m]',ylabel='world y [m]',xlim=(-.5,2.8),ylim=(-1.5,1.5));ax.set_aspect('equal');ax.legend(fontsize=7);ax.grid(alpha=.2)
fig.savefig(root/'physical_paths.png',dpi=160);plt.close(fig)
(root/'summary.json').write_text(json.dumps(rows,indent=2))
lines=['| Case | Status | Recoveries | Time s | Min body clearance m | Paired AMCL/world max m | World goal difference m |','|---|---:|---:|---:|---:|---:|---:|']
for r in rows:
    lines.append(f"| {r['case']} | {r['status']} | {r['recoveries']} | {r['wall_seconds']:.2f} | {r['minimum_sampled_footprint_clearance']:.3f} | {r['max_paired_amcl_world_error'] or 0:.3f} | {r['world_endpoint_goal_difference']:.3f} |")
(root/'measurements.md').write_text('\n'.join(lines)+'\n')
print(json.dumps(rows,indent=2))
