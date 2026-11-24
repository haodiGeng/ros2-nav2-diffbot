#!/usr/bin/env python3
from pathlib import Path
import json,sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(sys.argv[1]) if len(sys.argv)>1 else Path.home()/'nav_ws/artifacts/m7_tracking'
fig,axes=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
for variant,label in [('baseline','Original (5 recoveries)'),('prune06','Local horizon 0.6 m (0 recoveries)')]:
    folder=root/('box_astar_0_'+variant);t=json.loads((folder/'trial.json').read_text());d=json.loads((folder/'diagnostic.json').read_text())
    start=t['initial_world'][0]
    axes[0].plot([s['stamp']-start for s in d['scores']],[s['best']['vx'] for s in d['scores']],label=label,alpha=.8)
    axes[1].plot([s[0]-start for s in t['truth']],[s[1] for s in t['truth']],label=label)
axes[0].set(title='DWB selected forward velocity',xlabel='Simulation time since request [s]',ylabel='vx [m/s]')
axes[1].set(title='Recorded world position',xlabel='Simulation time since request [s]',ylabel='world x [m]')
for ax in axes:ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.savefig(root/'stall_comparison.png',dpi=160)
rows=[]
for variant in ['baseline','goalalign32','prune06']:
    folder=root/('box_astar_0_'+variant);t=json.loads((folder/'trial.json').read_text());d=json.loads((folder/'diagnostic.json').read_text())
    def distance(s):
        p=min(t['truth'],key=lambda p:abs(p[0]-s['stamp']))
        return ((p[1]-.48)**2+(p[2]+.5)**2)**.5
    nearest=min(d['scores'],key=distance)
    rows.append({'variant':variant,'status':t['status'],'recoveries':t['recoveries'],'wall_seconds':t['wall_seconds'],
        'nearest_score_to_stall_location':nearest,'sample_location_distance':distance(nearest)})
(root/'score_comparison.json').write_text(json.dumps(rows,indent=2))
