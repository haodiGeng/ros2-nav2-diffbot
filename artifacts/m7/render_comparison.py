"""Render recorded M7 evidence. Usage: python3 render_comparison.py [artifact_dir]."""
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).parent
report=json.loads((root/'planner_comparison.json').read_text())
grid=json.loads((root/'frozen_costmap.json').read_text())
w,h=grid['width'],grid['height'];r=grid['resolution'];ox,oy=grid['origin']
costs=np.asarray(grid['data']).reshape(h,w)
extent=(ox,ox+w*r,oy,oy+h*r)
fig,axes=plt.subplots(1,3,figsize=(15,5),constrained_layout=True)
groups=[('NavFn and self A*',['NavFnDijkstra','NavFnAStar','AStar8']),
        ('Connectivity',['AStar8','AStar4']),
        ('Cost and clearance',['AStarUnweighted','AStar8','AStarClearance'])]
for ax,(title,ids) in zip(axes,groups):
    ax.imshow(costs,origin='lower',extent=extent,cmap='Greys',vmin=0,vmax=254,alpha=.65)
    for key in ids:
        p=next(p for p in report['paths'] if p['pair']==0 and p['planner']==key)
        xy=np.asarray(p['points']);ax.plot(xy[:,0],xy[:,1],label=key,linewidth=1.8)
    ax.scatter([0,2.2],[0,0],color=['green','red'],s=30,zorder=5)
    ax.set(xlim=(-.6,2.6),ylim=(-1.4,1.4),title=title,xlabel='map x [m]',ylabel='map y [m]')
    ax.set_aspect('equal');ax.legend(fontsize=8,loc='upper left');ax.grid(alpha=.15)
fig.suptitle('M7: same frozen costmap, start (0,0), goal (2.2,0)')
fig.savefig(root/'planner_paths.png',dpi=160);plt.close(fig)

fig,axes=plt.subplots(1,2,figsize=(11,5),constrained_layout=True)
for name,color in [('astar','tab:blue'),('navfn','tab:orange')]:
    trials=json.loads((root/(name+'_results.json')).read_text())
    for i,t in enumerate(trials):
        xy=np.asarray(t['truth']);axes[0].plot(xy[:,1],xy[:,2],color=color,alpha=.7,label=name if i==0 else None)
    summary=json.loads((root/(name+'_results_summary.json')).read_text())
    axes[1].plot([1,2,3],[t['wall_seconds'] for t in summary],'o-',color=color,label=name)
axes[0].add_patch(plt.Rectangle((1.,-.2),.4,.4,color='black',label='test box'))
axes[0].set(title='Recorded physical trajectories',xlabel='world x [m]',ylabel='world y [m]')
axes[0].set_aspect('equal');axes[0].legend();axes[0].grid(alpha=.2)
axes[1].set(title='One navigation run each (not a statistical ranking)',xlabel='Goal sequence',ylabel='Wall time [s]',xticks=[1,2,3])
axes[1].legend();axes[1].grid(alpha=.2)
fig.savefig(root/'navigation_comparison.png',dpi=160);plt.close(fig)

lines=['# M7 fixed-map measurements','',
       'Lengths describe returned polylines; clearance is sampled center-to-known-lethal-cell-area distance.',
       'Planning time is the planner-server result duration, median of five measured runs after one warm-up.', '',
       '| Query | Planner | Time (ms) | Length (m) | Center clearance (m) | Expanded |',
       '|---|---|---:|---:|---:|---:|']
for s in report['summaries']:
    lines.append(f"| {s['pair']} | {s['planner']} | {s['median_planning_ms']:.3f} | {s['median_length']:.3f} | {s['sampled_min_center_obstacle_distance']:.3f} | {s['median_expanded'] if s['median_expanded'] is not None else 'N/A'} |")
(root/'comparison_table.md').write_text('\n'.join(lines)+'\n')
