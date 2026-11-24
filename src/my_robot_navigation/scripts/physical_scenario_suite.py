#!/usr/bin/env python3
"""Build matching SDF/maps, run fresh simulations sequentially, archive each trial."""
from pathlib import Path
import os,subprocess,signal,json,copy,sys,xml.etree.ElementTree as ET
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

root=Path.home()/'nav_ws';out=root/('artifacts/m7_gate_study' if '--gate-study' in sys.argv else ('artifacts/m7_width_study' if '--width-study' in sys.argv else ('artifacts/m7_tracking_regression' if '--tracking-regression' in sys.argv else 'artifacts/m7_physical')));out.mkdir(parents=True,exist_ok=True)
if '--gate-validation' in sys.argv:
    out=root/'artifacts/m7_gate_validation';out.mkdir(parents=True,exist_ok=True)
nav=Path(get_package_share_directory('my_robot_navigation'));sim=Path(get_package_share_directory('my_robot_gazebo'))
base=yaml.safe_load((nav/'config/nav2_astar_params.yaml').read_text())
script=root/'src/my_robot_navigation/scripts/physical_scenario_trial.py'
env=os.environ.copy();env.update(ROS_DOMAIN_ID='25',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
summary=json.loads((out/'suite.json').read_text()) if '--resume' in sys.argv and (out/'suite.json').exists() else []
cases=[('wide',p,i) for i in range(2) for p in ['astar','navfn']]+[('box',p,i) for i in range(2) for p in ['astar','navfn']]+[('narrow','clearance',0),('wide','clearance',0),('narrow','astar',0)]
if '--tracking-regression' in sys.argv:
    cases=[('box','astar',i) for i in range(2)]+[('wide','astar',i) for i in range(2)]+[('narrow','astar',i) for i in range(2)]+[('wide','clearance',0),('narrow','clearance',0),('box','navfn',0),('wide','navfn',0)]
if '--width-study' in sys.argv:
    cases=[(f'gap{width:03d}','astar',i) for width in [80,90,100,120] for i in range(2)]
if '--gate-study' in sys.argv:
    cases=[(f'gap{width:03d}','gate',i) for width in [80,100,120] for i in range(2)]
if '--gate-validation' in sys.argv:
    cases=[(f'gap{width:03d}','gate',i) for width in [80,100,120,140] for i in range(2)]
for scene,planner,rep in cases:
    folder=out/f'{scene}_{planner}_{rep}';folder.mkdir(exist_ok=True)
    if '--resume' in sys.argv and any(x['case']==folder.name and x['passed'] for x in summary):continue
    world=ET.parse(sim/'worlds/training.sdf');w=world.getroot().find('world')
    # Outer wall inner faces at +/-3.0, matching the analytical map exactly.
    for model in w.findall('model'):
        name=model.attrib['name']
        if name.startswith('wall_'):
            poses={'wall_n':'0 3.075 .5 0 0 0','wall_s':'0 -3.075 .5 0 0 0','wall_e':'3.075 0 .5 0 0 0','wall_w':'-3.075 0 .5 0 0 0'}
            model.find('pose').text=poses[name]
        if name=='test_box' and scene!='box':w.remove(model)
    obstacles=[[-3.2,3.,3.2,3.15],[-3.2,-3.15,3.2,-3.],[-3.15,-3.2,-3.,3.2],[3.,-3.2,3.15,3.2]]
    if scene=='box':obstacles.append([1.,-.2,1.4,.2])
    else:
        gap=int(scene[3:])/100 if scene.startswith('gap') else (1.5 if scene=='wide' else .8)
        for sign in [-1,1]:
            y=sign*(3+gap/2)/2;sy=3-gap/2
            m=ET.SubElement(w,'model',name='passage_'+str(sign));ET.SubElement(m,'static').text='true'
            ET.SubElement(m,'pose').text=f'1.3 {y} .5 0 0 0';link=ET.SubElement(m,'link',name='body')
            for kind in ['visual','collision']:
                obj=ET.SubElement(link,kind,name=kind);geom=ET.SubElement(obj,'geometry');box=ET.SubElement(geom,'box');ET.SubElement(box,'size').text=f'1.0 {sy} 1.0'
            obstacles.append([.8,min(sign*gap/2,sign*3),1.8,max(sign*gap/2,sign*3)])
    world.write(folder/'world.sdf',encoding='unicode')
    yy,xx=np.mgrid[0:128,0:128];x=-3.2+(xx+.5)*.05;y=-3.2+(yy+.5)*.05
    occupied=np.zeros((128,128),dtype=bool)
    for x0,y0,x1,y1 in obstacles:occupied |= (x>=x0)&(x<x1)&(y>=y0)&(y<y1)
    (folder/'map.pgm').write_bytes(b'P5\n128 128\n255\n'+np.flipud(np.where(occupied,0,254).astype(np.uint8)).tobytes())
    (folder/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[-3.2,-3.2,0.],negate=0,occupied_thresh=.65,free_thresh=.25)))
    cfg=copy.deepcopy(base);p=cfg['planner_server']['ros__parameters']
    if planner=='navfn':p['GridBased']=copy.deepcopy(p['NavFnBaseline'])
    elif planner=='clearance':p['GridBased']['min_obstacle_clearance']=.15
    (folder/'params.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    (folder/'geometry.json').write_text(json.dumps({'obstacles':obstacles,'map_world_alignment':'identity by construction','planner':planner,'scene':scene,'repetition':rep},indent=2))
    f=(folder/'navigation.log').open('w')
    launch=subprocess.Popen(['bash',str(root/'src/my_robot_navigation/scripts/start_navigation.sh'),'rviz:=false','gui:=false',
        'world_file:='+str(folder/'world.sdf'),'map:='+str(folder/'map.yaml'),'params_file:='+str(folder/'params.yaml')],stdout=f,stderr=f,start_new_session=True,env=env)
    try:
        mode=['gate'] if planner=='gate' else (['reject'] if planner=='clearance' and scene=='narrow' else [])
        response=subprocess.run(['python3',str(script),str(folder),*mode],env=env,timeout=240)
        assert response.returncode==0,'Trial process failed'
        trial=json.loads((folder/'trial.json').read_text())
        validation=trial['passed'];expected='success'
        if planner=='gate':
            if scene in ('gap080','gap100'):
                expected='preflight_rejection';validation=trial['passed'] and not trial['navigation_goal_sent'] and trial['preflight_status']==6
            elif scene=='gap120':
                expected='runtime_abort_and_hold';validation=(trial['navigation_goal_sent'] and trial['status']==6 and trial['recoveries']==0
                    and trial.get('max_translation_after_abort_settled',1)<.01 and trial.get('nonzero_commands_after_abort_settled',1)==0)
            else:validation=trial['passed'] and trial['navigation_goal_sent'] and trial['recoveries']==0
        summary.append({'case':folder.name,'passed':trial['passed'],'validation_passed':validation,'expected_outcome':expected,'status':trial.get('status')})
        (out/'suite.json').write_text(json.dumps(summary,indent=2))
        assert validation,'Trial did not meet expected outcome'
    finally:
        os.killpg(launch.pid,signal.SIGINT)
        try:launch.wait(timeout=20)
        except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
        f.close()
print('All physical scenarios completed',flush=True)
