#!/usr/bin/env python3
"""Own-robot navigation regression. Does not reset localization between goals."""
import json
import math
from pathlib import Path
import sys
import time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path as NavPath
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState

rclpy.init();n=rclpy.create_node('own_robot_navigation_trial')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
data={'odom':[],'truth':[],'amcl':[],'plans':[]}
def record(key,msg):
    p=msg.pose.pose;q=p.orientation
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    data[key].append([stamp,p.position.x,p.position.y,
                     math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
for key,topic,cls in [('odom','/odom',Odometry),('truth','/ground_truth',Odometry),
                       ('amcl','/amcl_pose',PoseWithCovarianceStamped)]:
    n.create_subscription(cls,topic,lambda m,k=key:record(k,m),qos_profile_sensor_data)
n.create_subscription(NavPath,'/plan',lambda m:data['plans'].append(
    [[p.pose.position.x,p.pose.position.y] for p in m.poses]),10)
client=ActionClient(n,NavigateToPose,'/navigate_to_pose')
out=Path.home()/'nav_ws/artifacts/m6';out.mkdir(parents=True,exist_ok=True)
filename=sys.argv[1] if len(sys.argv)>1 else 'navigation_results.json'
mode=sys.argv[2] if len(sys.argv)>2 else 'regression'
assert mode in ('smoke','avoidance','return','regression'),'Unknown test mode'
def wait(f,seconds):
    rclpy.spin_until_future_complete(n,f,timeout_sec=seconds)
    assert f.done(),'Request timeout'
    return f.result()
results=[];handle=None;pending=None
try:
    # Action servers can exist before lifecycle activation; require active state.
    for name in ('amcl','planner_server','controller_server','bt_navigator'):
        c=n.create_client(GetState,'/'+name+'/get_state')
        assert c.wait_for_service(timeout_sec=45),'Missing lifecycle '+name
        end=time.monotonic()+45
        while True:
            response=wait(c.call_async(GetState.Request()),5)
            if response.current_state.id==3:break
            assert time.monotonic()<end,'Inactive '+name
            rclpy.spin_once(n,timeout_sec=.2)
    assert client.wait_for_server(timeout_sec=15)
    end=time.monotonic()+15
    while not data['truth'] and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
    assert data['truth'],'No physical pose'
    goals={'smoke':[(-.6,0.,0.)], 'avoidance':[(2.,0.,0.)],
           'return':[(0.,0.,0.),(0.,1.6,0.)],
           'regression':[(2.,0.,0.),(0.,0.,0.),(0.,1.6,0.)]}[mode]
    for x,y,yaw in goals:
        starts={key:len(value) for key,value in data.items()}
        counts=[]
        g=NavigateToPose.Goal();g.pose.header.frame_id='map'
        g.pose.header.stamp=n.get_clock().now().to_msg()
        g.pose.pose.position.x=x;g.pose.pose.position.y=y
        g.pose.pose.orientation.z=math.sin(yaw/2);g.pose.pose.orientation.w=math.cos(yaw/2)
        handle=wait(client.send_goal_async(g,feedback_callback=lambda m:counts.append(m.feedback.number_of_recoveries)),15)
        assert handle.accepted,'Goal rejected'
        start=time.monotonic();pending=handle.get_result_async()
        result=wait(pending,150)
        trial={'goal':[x,y,yaw],'status':result.status,'wall_seconds':time.monotonic()-start,
               'recoveries':max(counts,default=0)}
        for key,value in data.items():trial[key]=value[starts[key]:]
        results.append(trial)
        (out/filename).write_text(json.dumps(results))
        print({k:v for k,v in trial.items() if k not in data},flush=True)
        if result.status!=4:break
        handle=None;pending=None
finally:
    if handle is not None and pending is not None and not pending.done():
        f=handle.cancel_goal_async();rclpy.spin_until_future_complete(n,f,timeout_sec=10)
    n.destroy_node();rclpy.shutdown()
