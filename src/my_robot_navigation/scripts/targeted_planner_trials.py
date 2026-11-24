#!/usr/bin/env python3
"""Synthetic maps, real Nav2 plugins, isolated domain 28; never sends cmd_vel."""
import os
os.environ.update(ROS_DOMAIN_ID='28',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
from pathlib import Path
import json, hashlib, math, subprocess, time
import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import GetCostmap
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

out=Path.home()/'nav_ws/artifacts/m7_targeted';out.mkdir(parents=True,exist_ok=True)
share=Path(get_package_share_directory('my_robot_navigation'))
base=yaml.safe_load((share/'config/planner_benchmark.yaml').read_text())
ids=base['planner_server']['ros__parameters']['planner_plugins']
report={'passed':False,'domain':28,'scenarios':[]}

def scenario(name):
    g=np.zeros((120,120),dtype=np.uint8);g[[0,-1],:]=254;g[:,[0,-1]]=254
    if name=='diagonal_pinch':
        # Opposite free quadrants touch only at one corner. Point-robot topology test.
        g[:60,60:]=254;g[60:,:60]=254
        return g,(2.025,2.025),(4.025,4.025),0.,0.
    if name.startswith('corridor'):
        gap=16 if name.endswith('narrow') else 30
        low=60-gap//2;high=low+gap
        g[:low,:]=254;g[high:,:]=254
        return g,(1.025,3.025),(4.975,3.025),.35,(.35 if name.endswith('narrow') else .65)
    # Thin rectangular obstacle across the direct route; soft preference comparison.
    g[45:75,55:65]=254
    return g,(1.525,3.025),(4.475,3.025),.35,1.0

rclpy.init();n=rclpy.create_node('targeted_planner_trials')
def wait(f,seconds=25):
    rclpy.spin_until_future_complete(n,f,timeout_sec=seconds)
    assert f.done() and f.exception() is None,'ROS request timeout'
    return f.result()
try:
    for name in ['diagonal_pinch','corridor_narrow','corridor_wide','wall_preference']:
        folder=out/name;folder.mkdir(exist_ok=True);processes=[];logs=[]
        g,start,goal,radius,inflation=scenario(name)
        # PGM top row is world north; save inverse y. 254 here denotes occupied.
        pixels=np.where(g==254,0,254).astype(np.uint8)
        (folder/'map.pgm').write_bytes(b'P5\n120 120\n255\n'+np.flipud(pixels).tobytes())
        (folder/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[0.,0.,0.],negate=0,occupied_thresh=.65,free_thresh=.25)))
        cfg=yaml.safe_load(yaml.safe_dump(base))
        cost=cfg['global_costmap']['global_costmap']['ros__parameters']
        cost['robot_radius']=radius;cost['inflation_layer']['inflation_radius']=inflation
        cfg['map_server']['ros__parameters']['yaml_filename']=str(folder/'map.yaml')
        (folder/'params.yaml').write_text(yaml.safe_dump(cfg))
        def spawn(pkg,exe,args):
            f=(folder/(exe+'.log')).open('w');logs.append(f)
            p=Path(get_package_prefix(pkg))/'lib'/pkg/exe
            processes.append(subprocess.Popen([str(p),*args],stdout=f,stderr=f))
        try:
            spawn('tf2_ros','static_transform_publisher',['--x','1.025','--y','3.025','--z','0','--frame-id','map','--child-frame-id','base_footprint'])
            for pkg,exe in [('nav2_map_server','map_server'),('nav2_planner','planner_server')]:
                spawn(pkg,exe,['--ros-args','--params-file',str(folder/'params.yaml')])
                c=n.create_client(ChangeState,'/'+exe+'/change_state');assert c.wait_for_service(timeout_sec=25)
                for transition in [1,3]:
                    req=ChangeState.Request();req.transition.id=transition
                    assert wait(c.call_async(req)).success
                n.destroy_client(c)
            c=n.create_client(GetCostmap,'/global_costmap/get_costmap');assert c.wait_for_service(timeout_sec=15)
            deadline=time.monotonic()+20
            while True:
                m=wait(c.call_async(GetCostmap.Request())).map
                if m.metadata.size_x==120 and 254 in m.data:break
                assert time.monotonic()<deadline
                rclpy.spin_once(n,timeout_sec=.2)
            # Static-layer arrival and the first inflation update are asynchronous.
            stable=0;previous=None;deadline=time.monotonic()+20
            while stable<3:
                rclpy.spin_once(n,timeout_sec=.6)
                m=wait(c.call_async(GetCostmap.Request())).map
                current=bytes(m.data)
                stable=stable+1 if current==previous else 0;previous=current
                assert time.monotonic()<deadline,'Costmap never stabilized'
            raw=bytes(m.data);digest=hashlib.sha256(raw).hexdigest()
            grid=np.asarray(m.data).reshape(120,120)
            (folder/'costmap.json').write_text(json.dumps({'data':list(m.data),'sha256':digest,'resolution':.05,'origin':[0,0],'width':120,'height':120}))
            client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');assert client.wait_for_server(timeout_sec=15)
            # NavFn clears its start cell. Normalize that single cell once BEFORE
            # timing any planner, then require an identical grid for every request.
            q=ComputePathToPose.Goal();q.planner_id='NavFnDijkstra';q.use_start=True
            for pose,xy in [(q.start,start),(q.goal,goal)]:
                pose.header.frame_id='map';pose.pose.position.x=xy[0];pose.pose.position.y=xy[1];pose.pose.orientation.w=1.
            handle=wait(client.send_goal_async(q));wait(handle.get_result_async())
            normalized=bytes(wait(c.call_async(GetCostmap.Request())).map.data)
            changed=[i for i,(a,b) in enumerate(zip(raw,normalized)) if a!=b]
            start_cell=int(start[1]/.05)*120+int(start[0]/.05)
            assert all(i==start_cell and normalized[i]==0 for i in changed),'Unexpected normalization change'
            raw=normalized;digest=hashlib.sha256(raw).hexdigest();grid=np.frombuffer(raw,dtype=np.uint8).reshape(120,120)
            (folder/'costmap.json').write_text(json.dumps({'data':list(raw),'sha256':digest,'resolution':.05,'origin':[0,0],'width':120,'height':120,'normalized_start_cells':changed}))
            rows=[]
            for key in ids:
                for repetition in range(4):
                    q=ComputePathToPose.Goal();q.planner_id=key;q.use_start=True
                    for pose,xy in [(q.start,start),(q.goal,goal)]:
                        pose.header.frame_id='map';pose.pose.position.x=xy[0];pose.pose.position.y=xy[1];pose.pose.orientation.w=1.
                    handle=wait(client.send_goal_async(q));assert handle.accepted
                    response=wait(handle.get_result_async())
                    after=bytes(wait(c.call_async(GetCostmap.Request())).map.data)
                    assert hashlib.sha256(after).hexdigest()==digest,('Costmap mutated',key,[(i%120,i//120,a,b) for i,(a,b) in enumerate(zip(raw,after)) if a!=b][:12])
                    points=[[p.pose.position.x,p.pose.position.y] for p in response.result.path.poses]
                    if repetition==0:continue
                    row={'planner':key,'repetition':repetition,'status':response.status,'points':points,
                         'length':sum(math.dist(a,b) for a,b in zip(points,points[1:])),
                         'planning_ms':1000*response.result.planning_time.sec+response.result.planning_time.nanosec/1e6}
                    samples=[]
                    for a,b in zip(points,points[1:]):
                        steps=max(1,math.ceil(math.dist(a,b)/.0025))
                        samples.extend([[a[0]+(b[0]-a[0])*j/steps,a[1]+(b[1]-a[1])*j/steps] for j in range(steps+1)])
                    if samples:
                        samples=np.asarray(samples);yy,xx=np.where(grid==254)
                        distance=math.inf
                        for batch in np.array_split(samples,max(1,len(samples)//128)):
                            dx=np.maximum(np.abs(batch[:,0,None]-(xx+.5)*.05)-.025,0)
                            dy=np.maximum(np.abs(batch[:,1,None]-(yy+.5)*.05)-.025,0)
                            distance=min(distance,float(np.hypot(dx,dy).min()))
                        row['sampled_center_clearance']=distance
                    rows.append(row)
            entry={'name':name,'robot_radius':radius,'inflation_radius':inflation,'costmap_sha256':digest,'start':start,'goal':goal,'rows':rows}
            report['scenarios'].append(entry)
            fig,ax=plt.subplots(figsize=(7,6));ax.imshow(grid,origin='lower',extent=(0,6,0,6),cmap='Greys',vmin=0,vmax=254)
            for key in ids:
                row=next(x for x in rows if x['planner']==key)
                if row['points']:
                    xy=np.array(row['points']);ax.plot(xy[:,0],xy[:,1],label=key,lw=1.5)
            ax.scatter([start[0],goal[0]],[start[1],goal[1]],c=['green','red']);ax.set_title(name+f' (radius={radius} m)')
            if any(x['points'] for x in rows):ax.legend(fontsize=7)
            ax.set(xlabel='map x [m]',ylabel='map y [m]');fig.tight_layout();fig.savefig(folder/'paths.png',dpi=140);plt.close(fig)
            print(name,[(key,next((x['status'],x.get('sampled_center_clearance')) for x in rows if x['planner']==key)) for key in ids],flush=True)
            client.destroy();n.destroy_client(c)
        finally:
            for p in reversed(processes):
                p.terminate()
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:p.kill();p.wait()
            for f in logs:f.close()
        (out/'results.json').write_text(json.dumps(report,indent=2))
    by={x['name']:x for x in report['scenarios']}
    assert all(x['status']==6 and not x['points'] for x in by['diagonal_pinch']['rows'] if x['planner'].startswith('AStar'))
    assert all(x['status']==(6 if x['planner']=='AStarClearance' else 4) for x in by['corridor_narrow']['rows'])
    assert all(x['status']==4 for x in by['corridor_wide']['rows'])
    assert all(x['status']==4 for x in by['wall_preference']['rows'])
    report['passed']=True
    (out/'results.json').write_text(json.dumps(report,indent=2))
finally:
    n.destroy_node();rclpy.shutdown()
