#!/usr/bin/env python3
"""Isolated static-map planner benchmark; no simulated robot is driven.

ROS domain 27 has a synthetic fixed TF only for planner-server initialization.
All plans use explicit start poses. Hash the raw costmap after EVERY request.
"""
from pathlib import Path
import hashlib
import json
import math
import os
import statistics
import subprocess
import time
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory,get_package_prefix
import rclpy
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import GetCostmap
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from std_msgs.msg import String

os.environ['ROS_DOMAIN_ID']='27'
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
out=Path.home()/'nav_ws/artifacts/m7';out.mkdir(parents=True,exist_ok=True)
share=Path(get_package_share_directory('my_robot_navigation'))
cfg=share/'config/planner_benchmark.yaml'
config=yaml.safe_load(cfg.read_text())
ids=config['planner_server']['ros__parameters']['planner_plugins']
processes=[];logs=[]
def spawn(package,exe,args):
    log=(out/('benchmark_'+exe+'.log')).open('w');logs.append(log)
    path=Path(get_package_prefix(package))/'lib'/package/exe
    p=subprocess.Popen([str(path),*args],stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
    processes.append(p)
def duration(value):return value.sec+value.nanosec*1e-9
rows=[];report={'passed':False,'domain':27,'variants':ids,'parameters':config['planner_server']['ros__parameters']}
try:
    spawn('tf2_ros','static_transform_publisher',['--x','0','--y','0','--z','0',
          '--roll','0','--pitch','0','--yaw','0','--frame-id','map','--child-frame-id','base_footprint'])
    spawn('nav2_map_server','map_server',['--ros-args','--params-file',str(cfg),
          '-p','yaml_filename:='+str(share/'maps/my_robot_training.yaml')])
    spawn('nav2_planner','planner_server',['--ros-args','--params-file',str(cfg)])
    rclpy.init();n=rclpy.create_node('fixed_map_planner_comparison')
    stats={key:[] for key in ids}
    for key in ids:
        n.create_subscription(String,'/planner_server/'+key+'/statistics',
            lambda m,k=key:stats[k].append(json.loads(m.data)),10)
    def wait(f,timeout=15):
        rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
        assert f.done() and f.exception() is None,'Request timeout or exception'
        return f.result()
    for name in ('map_server','planner_server'):
        client=n.create_client(ChangeState,'/'+name+'/change_state')
        assert client.wait_for_service(timeout_sec=20),'No lifecycle service '+name
        for transition in (1,3):
            req=ChangeState.Request();req.transition.id=transition
            assert wait(client.call_async(req),30).success,'Lifecycle failure '+name
    getmap=n.create_client(GetCostmap,'/global_costmap/get_costmap')
    assert getmap.wait_for_service(timeout_sec=15)
    deadline=time.monotonic()+20
    while True:
        m=wait(getmap.call_async(GetCostmap.Request())).map
        if m.metadata.size_x>100 and sum(v==0 for v in m.data)>1000:break
        assert time.monotonic()<deadline,'Static costmap not ready'
        rclpy.spin_once(n,timeout_sec=.2)
    raw=bytes(m.data);fingerprint=hashlib.sha256(raw).hexdigest()
    w,h=m.metadata.size_x,m.metadata.size_y;r=m.metadata.resolution
    ox,oy=m.metadata.origin.position.x,m.metadata.origin.position.y
    grid=np.asarray(m.data,dtype=np.uint8).reshape(h,w)
    (out/'frozen_costmap.json').write_text(json.dumps(dict(width=w,height=h,resolution=r,origin=[ox,oy],data=list(m.data),sha256=fingerprint)))
    report['costmap_sha256']=fingerprint
    obstacle_y,obstacle_x=np.where(grid==254)
    obstacle_x=ox+(obstacle_x+.5)*r;obstacle_y=oy+(obstacle_y+.5)*r
    def geometry(points):
        sampled=[]
        for a,b in zip(points,points[1:]):
            steps=max(1,math.ceil(math.dist(a,b)/(r/10)))
            sampled.extend([[a[0]+(b[0]-a[0])*j/steps,a[1]+(b[1]-a[1])*j/steps] for j in range(steps)])
        sampled.append(points[-1]);sampled=np.asarray(sampled)
        x=np.floor((sampled[:,0]-ox)/r).astype(int);y=np.floor((sampled[:,1]-oy)/r).astype(int)
        inside=(x>=0)&(x<w)&(y>=0)&(y<h)
        safe=bool(inside.all())
        if safe:safe=bool((grid[y,x]<253).all())
        minimum=math.inf
        if len(obstacle_x):
            for begin in range(0,len(sampled),256):
                p=sampled[begin:begin+256]
                dx=np.maximum(np.abs(p[:,0,None]-obstacle_x[None,:])-r/2,0.)
                dy=np.maximum(np.abs(p[:,1,None]-obstacle_y[None,:])-r/2,0.)
                minimum=min(minimum,float(np.hypot(dx,dy).min()))
        return {'sampled_base_costmap_safe':safe,'sampled_min_center_obstacle_distance':minimum if math.isfinite(minimum) else None}
    client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
    assert client.wait_for_server(timeout_sec=15)
    pairs=[([0.,0.],[2.2,0.]),([-1.8,-1.8],[1.8,1.8]),([0.,-1.8],[2.2,1.8])]
    representative={}
    for pair_id,(start,goal) in enumerate(pairs):
        for key in ids:
            start_index=int((start[1]-oy)/r)*w+int((start[0]-ox)/r)
            assert raw[start_index]<253,'Benchmark start blocked'
            for repetition in range(6):
                request=ComputePathToPose.Goal();request.planner_id=key;request.use_start=True
                for pose,xy in ((request.start,start),(request.goal,goal)):
                    pose.header.frame_id='map';pose.pose.position.x=xy[0];pose.pose.position.y=xy[1];pose.pose.orientation.w=1.
                before=len(stats[key]);t=time.perf_counter()
                handle=wait(client.send_goal_async(request));assert handle.accepted
                response=wait(handle.get_result_async());latency=time.perf_counter()-t
                # Drain independent statistics topic without pairing a previous request's data.
                until=time.monotonic()+.02
                while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.005)
                points=[[p.pose.position.x,p.pose.position.y] for p in response.result.path.poses]
                assert response.status==4 and points,'Planning failed '+key+' pair '+str(pair_id)
                if key.startswith('AStar'):
                    assert math.dist(points[0],start)<1e-9 and math.dist(points[-1],goal)<1e-9,'Endpoint changed'
                after=wait(getmap.call_async(GetCostmap.Request())).map
                assert hashlib.sha256(bytes(after.data)).hexdigest()==fingerprint,'Costmap changed during benchmark'
                if repetition==0:continue
                item={'planner':key,'pair':pair_id,'start':start,'goal':goal,'repetition':repetition,
                      'status':response.status,'planning_seconds':duration(response.result.planning_time),
                      'action_round_trip_seconds':latency,
                      'length':sum(math.dist(a,b) for a,b in zip(points,points[1:])),
                      'pose_count':len(points)}
                if len(stats[key])>before:item['statistics']=stats[key][-1]
                rows.append(item)
                if (pair_id,key) not in representative:
                    representative[(pair_id,key)]={'pair':pair_id,'planner':key,'points':points,**geometry(points)}
        print('Compared frozen-map query',pair_id,flush=True)
    summaries=[]
    for pair in range(len(pairs)):
        for key in ids:
            group=[x for x in rows if x['pair']==pair and x['planner']==key]
            summary={'pair':pair,'planner':key,'runs':len(group),
              'median_planning_ms':statistics.median(x['planning_seconds'] for x in group)*1000,
              'median_action_round_trip_ms':statistics.median(x['action_round_trip_seconds'] for x in group)*1000,
              'median_length':statistics.median(x['length'] for x in group),
              **{k:v for k,v in representative[(pair,key)].items() if k not in ('points','pair','planner')}}
            expanded=[x['statistics']['expanded'] for x in group if 'statistics' in x]
            summary['median_expanded']=statistics.median(expanded) if expanded else None
            summaries.append(summary)
    assert all(p['sampled_base_costmap_safe'] for p in representative.values()),'Unsafe sampled path'
    rejected=[]
    for goal in ([1.2,0.],[20.,20.]):
        request=ComputePathToPose.Goal();request.planner_id='AStar8';request.use_start=True
        for pose,xy in ((request.start,[0.,0.]),(request.goal,goal)):
            pose.header.frame_id='map';pose.pose.position.x=xy[0];pose.pose.position.y=xy[1];pose.pose.orientation.w=1.
        handle=wait(client.send_goal_async(request));assert handle.accepted
        response=wait(handle.get_result_async())
        assert response.status==6 and not response.result.path.poses,'Invalid goal returned a path'
        rejected.append({'goal':goal,'status':response.status,'pose_count':len(response.result.path.poses)})
    report.update(passed=True,costmap_unchanged=True,queries=pairs,measured_rows=rows,rejected_goals=rejected,
                  summaries=summaries,paths=list(representative.values()))
    print(json.dumps(summaries),flush=True)
except Exception as exc:
    report.update(error=str(exc),measured_rows=rows)
    raise
finally:
    (out/'planner_comparison.json').write_text(json.dumps(report,indent=2))
    for p in reversed(processes):
        p.terminate()
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:p.kill();p.wait()
    for log in logs:log.close()
    if rclpy.ok():rclpy.shutdown()
