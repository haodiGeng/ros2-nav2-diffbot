#!/usr/bin/env python3
"""One physical scenario trial. World truth is measurement only, never control."""
import json,math,time,sys
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import PoseWithCovarianceStamped,Twist,PoseStamped
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from nav2_msgs.srv import GetCostmap
from tf2_ros import Buffer,TransformListener

folder=Path(sys.argv[1]);reject=len(sys.argv)>2 and sys.argv[2]=='reject'
gated=len(sys.argv)>2 and sys.argv[2]=='gate'
rclpy.init();n=rclpy.create_node('physical_scenario_trial')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
data={'truth':[],'amcl':[],'odom':[],'commands':[],'plans':[],'planner_events':[],'failure_costmaps':[]};handle=None;pending=None
tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,n)
getmap=n.create_client(GetCostmap,'/global_costmap/get_costmap')
def planner_event(m):
    event={'stamp':n.get_clock().now().nanoseconds*1e-9,**json.loads(m.data)}
    data['planner_events'].append(event)
    if event.get('success') is False and len(data['failure_costmaps'])<3:
        f=getmap.call_async(GetCostmap.Request())
        def receive(future):
            try:
                grid=future.result().map;meta=grid.metadata
                sample={'event':event,'sample_stamp':n.get_clock().now().nanoseconds*1e-9,
                    'width':meta.size_x,'height':meta.size_y,'resolution':meta.resolution,
                    'origin':[meta.origin.position.x,meta.origin.position.y],'data':list(grid.data),
                    'truth':data['truth'][-1] if data['truth'] else None,'amcl':data['amcl'][-1] if data['amcl'] else None}
                try:
                    p=tf_buffer.lookup_transform('map','base_footprint',rclpy.time.Time()).transform.translation
                    x=int(math.floor((p.x-meta.origin.position.x)/meta.resolution));y=int(math.floor((p.y-meta.origin.position.y)/meta.resolution))
                    sample.update(map_base_xy=[p.x,p.y],cell_xy=[x,y],cell_cost=grid.data[y*meta.size_x+x] if 0<=x<meta.size_x and 0<=y<meta.size_y else None)
                except Exception as e:sample['tf_error']=str(e)
                data['failure_costmaps'].append(sample)
            except Exception as e:data['failure_costmaps'].append({'error':str(e)})
        f.add_done_callback(receive)
n.create_subscription(String,'/planner_server/GridBased/statistics',planner_event,10)
n.create_subscription(String,'/planner_server/ClearanceGate/statistics',planner_event,10)
n.create_subscription(NavPath,'/plan',lambda m:data['plans'].append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
    'points':[[p.pose.position.x,p.pose.position.y] for p in m.poses]}),10)
def pose(key,m):
    p=m.pose.pose;q=p.orientation
    data[key].append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.position.x,p.position.y,
        math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
for key,topic,kind in [('truth','/ground_truth',Odometry),('odom','/odom',Odometry),('amcl','/amcl_pose',PoseWithCovarianceStamped)]:
    n.create_subscription(kind,topic,lambda m,k=key:pose(k,m),qos_profile_sensor_data)
n.create_subscription(Twist,'/cmd_vel',lambda m:data['commands'].append([n.get_clock().now().nanoseconds*1e-9,m.linear.x,m.angular.z]),10)
def wait(f,limit):
    rclpy.spin_until_future_complete(n,f,timeout_sec=limit)
    assert f.done() and f.exception() is None,'Request timeout'
    return f.result()
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
result={'passed':False,'goal':[2.4,0.,0.],'reject_test':reject}
try:
    for name in ['amcl','planner_server','controller_server','bt_navigator']:
        c=n.create_client(GetState,'/'+name+'/get_state');assert c.wait_for_service(timeout_sec=45)
        end=time.monotonic()+45
        while wait(c.call_async(GetState.Request()),5).current_state.id!=3:
            assert time.monotonic()<end;spin(.2)
        n.destroy_client(c)
    spin(3)
    assert data['truth'] and data['amcl'],'No pose stream'
    result['initial_world']=data['truth'][-1];result['initial_amcl']=data['amcl'][-1]
    result['trial_start_index']={k:len(v) for k,v in data.items()}
    began=time.monotonic()
    if gated:
        from safe_navigate import GatedNavigator
        goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x=2.4;goal.pose.orientation.w=1.
        result.update(GatedNavigator(n).execute(goal))
        reject=not result['navigation_goal_sent'];result['reject_test']=reject
        result['wall_seconds']=time.monotonic()-began
    else:
        kind=ComputePathToPose if reject else NavigateToPose
        client=ActionClient(n,kind,'/compute_path_to_pose' if reject else '/navigate_to_pose')
        assert client.wait_for_server(timeout_sec=15)
        q=kind.Goal()
        target=q.goal if reject else q.pose
        target.header.frame_id='map';target.header.stamp=n.get_clock().now().to_msg()
        target.pose.position.x=2.4;target.pose.orientation.w=1.
        if reject:q.planner_id='GridBased';q.use_start=False
        counts=[]
        handle=wait(client.send_goal_async(q,feedback_callback=None if reject else lambda m:counts.append(m.feedback.number_of_recoveries)),15)
        assert handle.accepted
        pending=handle.get_result_async();response=wait(pending,150)
        result.update(status=response.status,wall_seconds=time.monotonic()-began,recoveries=max(counts,default=0))
        if reject:result['path_points']=len(response.result.path.poses)
    if gated and result['navigation_goal_sent'] and result['status']!=4:
        result['runtime_aborted']=True
        spin(1)
        anchor=data['truth'][-1];index=len(data['truth']);command_index=len(data['commands'])
        spin(4)
        result['max_translation_after_abort_settled']=max(math.hypot(p[1]-anchor[1],p[2]-anchor[2]) for p in data['truth'][index:])
        result['nonzero_commands_after_abort_settled']=sum(abs(p[1])>1e-5 or abs(p[2])>1e-5 for p in data['commands'][command_index:])
    else:spin(5 if reject else 1)
    assert len(data['truth'])>10
    result['final_world']=data['truth'][-1];result['final_amcl']=data['amcl'][-1]
    result['world_endpoint_goal_difference']=math.hypot(data['truth'][-1][1]-2.4,data['truth'][-1][2])
    result['passed']=result['status']==(6 if reject else 4)
    if reject:
        p=result['initial_world'];samples=data['truth'][result['trial_start_index']['truth']:]
        movement=max(math.hypot(x[1]-p[1],x[2]-p[2]) for x in samples)
        result['max_translation_during_rejection']=movement
        cmds=data['commands'][result['trial_start_index']['commands']:]
        result['nonzero_commands']=sum(abs(x[1])>1e-5 or abs(x[2])>1e-5 for x in cmds)
        result['passed'] &= movement<.01 and result['nonzero_commands']==0 and result['path_points']==0
except Exception as e:
    result['error']=str(e)
    raise
finally:
    if handle is not None and pending is not None and not pending.done():
        wait(handle.cancel_goal_async(),10)
    result.update(data)
    (folder/'trial.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in data}),flush=True)
    n.destroy_node();rclpy.shutdown()
