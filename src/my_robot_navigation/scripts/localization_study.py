#!/usr/bin/env python3
"""Fresh-world translation, rotation and navigation diagnostics. Truth never localizes robot."""
import os
os.environ.update(ROS_DOMAIN_ID='25',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
from pathlib import Path
import json,math,sys,time,signal,subprocess,atexit
import yaml,xml.etree.ElementTree as ET
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
from geometry_msgs.msg import Twist,PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path as NavPath
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from safe_navigate import GatedNavigator

mode=sys.argv[1];tag=sys.argv[2] if len(sys.argv)>2 else 'baseline'
assert mode in ('straight','rotation','loop')
r=Path.home()/'nav_ws';folder=r/'artifacts/m7_localization'/f'{tag}_{mode}';folder.mkdir(parents=True,exist_ok=True)
scene=r/'artifacts/m7_continuous_v2';args=[]
if len(sys.argv)>3 and sys.argv[3]!='-':args=['params_file:='+sys.argv[3]]
environment=os.environ.copy()
if len(sys.argv)>4:environment['MY_ROBOT_DRIVE_TRACK_SCALE']=sys.argv[4]
profile=Path(sys.argv[3]) if len(sys.argv)>3 and sys.argv[3]!='-' else r/'src/my_robot_navigation/config/nav2_astar_params.yaml'
(folder/'params.yaml').write_bytes(profile.read_bytes())
assert (scene/'map.yaml').is_file() and (scene/'world.sdf').is_file()
log=(folder/'navigation.log').open('w')
process=subprocess.Popen(['bash',str(r/'src/my_robot_navigation/scripts/start_astar_navigation.sh'),'rviz:=false','gui:=false',
 'map:='+str(scene/'map.yaml'),'world_file:='+str(scene/'world.sdf'),*args],stdout=log,stderr=log,start_new_session=True,env=environment)
cleanup_done=False
def stop_process():
 global cleanup_done
 if cleanup_done:return
 cleanup_done=True
 try:os.killpg(process.pid,signal.SIGINT)
 except ProcessLookupError:pass
 try:process.wait(timeout=20)
 except subprocess.TimeoutExpired:
  try:os.killpg(process.pid,signal.SIGTERM)
  except ProcessLookupError:pass
  process.wait(timeout=10)
 log.close()
atexit.register(stop_process)
rclpy.init();n=rclpy.create_node('localization_study');n.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
buffer=Buffer();listener=TransformListener(buffer,n)
data={'truth':[],'odom':[],'amcl':[],'synchronized_tf':[],'phases':[],'tf_failures':0,'mode':mode,'tag':tag}
data['plans']=[]
n.create_subscription(NavPath,'/plan',lambda m:data['plans'].append({
 'stamp':n.get_clock().now().nanoseconds*1e-9,
 'points':[[p.pose.position.x,p.pose.position.y] for p in m.poses]}),10)
data['requested_track_scale']=environment.get('MY_ROBOT_DRIVE_TRACK_SCALE','1.0')
data['scan_timing']=[];data['tf_failure_stamps']=[]
n.create_subscription(LaserScan,'/scan',lambda m:data['scan_timing'].append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
 n.get_clock().now().nanoseconds*1e-9,m.header.frame_id]),qos_profile_sensor_data)
pending=[]
def xyq(p):
 q=p.orientation
 return [p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
def receive(key,m):
 stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 row=[stamp,*xyq(m.pose.pose)];data[key].append(row)
 data.setdefault('frames',{})[key]=[m.header.frame_id,getattr(m,'child_frame_id','')]
 if key=='truth':pending.append(row)
 if key=='amcl':data.setdefault('amcl_covariance',[]).append([stamp,m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[35]])
for key,topic,kind in [('truth','/ground_truth',Odometry),('odom','/odom',Odometry),('amcl','/amcl_pose',PoseWithCovarianceStamped)]:
 n.create_subscription(kind,topic,lambda m,k=key:receive(k,m),qos_profile_sensor_data)
def transform(target,source,stamp):
 m=buffer.lookup_transform(target,source,Time(seconds=stamp)).transform;q=m.rotation
 return [m.translation.x,m.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
def synchronize():
 now=n.get_clock().now().nanoseconds*1e-9
 while pending and pending[0][0]<now-.3:
  row=pending.pop(0)
  try:data['synchronized_tf'].append({'stamp':row[0],'truth':row[1:],'map_base':transform('map','base_footprint',row[0]),
    'odom_base':transform('odom','base_footprint',row[0]),'map_odom':transform('map','odom',row[0])})
  except Exception:
   data['tf_failures']+=1;data['tf_failure_stamps'].append(row[0])
n.create_timer(.1,synchronize)
pub=n.create_publisher(Twist,'/cmd_vel',10)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
def stamp():return n.get_clock().now().nanoseconds*1e-9
def drive(label,v,w,duration):
 phase={'label':label,'start':stamp()};begin=stamp();deadline=time.monotonic()+duration*3+15
 while stamp()-begin<duration:
  assert time.monotonic()<deadline,'Simulation clock stalled'
  if mode=='straight' and data['truth']:
   pose=data['truth'][-1];assert -.35<pose[1]<1.2 and abs(pose[2])<.25,'Translation guard exceeded'
  m=Twist();m.linear.x=v;m.angular.z=w;pub.publish(m);rclpy.spin_once(n,timeout_sec=.05)
 pub.publish(Twist());spin(2);phase['end']=stamp();data['phases'].append(phase)
try:
 for name in ['amcl','planner_server','controller_server','bt_navigator']:
  c=n.create_client(GetState,'/'+name+'/get_state');assert c.wait_for_service(timeout_sec=45);end=time.monotonic()+45
  while True:
   f=c.call_async(GetState.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5)
   if f.done() and f.result().current_state.id==3:break
   assert time.monotonic()<end
 spin(4);assert data['truth'] and data['amcl']
 params=n.create_client(GetParameters,'/robot_state_publisher/get_parameters');assert params.wait_for_service(timeout_sec=5)
 req=GetParameters.Request();req.names=['robot_description'];future=params.call_async(req);rclpy.spin_until_future_complete(n,future,timeout_sec=5)
 description=future.result().values[0].string_value;(folder/'robot_description.xml').write_text(description)
 robot=ET.fromstring(description)
 data['effective_drive_separation']=float(robot.find('.//plugin/wheel_separation').text)
 data['wheel_joint_origins']={j.get('name'):j.find('origin').get('xyz') for j in robot.findall('joint') if j.get('name') in ['left_wheel_joint','right_wheel_joint']}
 laser=buffer.lookup_transform('base_footprint','laser_link',Time()).transform
 data['laser_translation']=[laser.translation.x,laser.translation.y,laser.translation.z]
 data['initial_truth']=data['truth'][-1]
 if mode=='straight':
  drive('forward',.12,0.,6.);drive('backward',-.12,0.,6.)
 elif mode=='rotation':
  drive('rotate_ccw',0.,.3,2*math.pi/.3);drive('rotate_cw',0.,-.3,2*math.pi/.3)
 else:
  gate=GatedNavigator(n)
  for label,x,y in [('around_box',0.,2.6),('return',0.,0.),('wide',3.3,0.),('return_again',0.,0.)]:
   q=PoseStamped();q.header.frame_id='map';q.pose.position.x=x;q.pose.position.y=y;q.pose.orientation.w=1.
   phase={'label':label,'start':stamp(),'goal':[x,y]};phase.update(gate.execute(q));phase['end']=stamp();data['phases'].append(phase)
   assert phase['status']==4
 spin(2);data['completed']=True
except Exception as e:
 data['error']=str(e);raise
finally:
 if mode!='loop':pub.publish(Twist());spin(.3)
 (folder/'record.json').write_text(json.dumps(data,indent=2))
 print(json.dumps({'mode':mode,'tag':tag,'completed':data.get('completed',False),'truth_samples':len(data['truth']),
   'tf_samples':len(data['synchronized_tf']),'tf_failures':data['tf_failures'],'phases':data['phases']}),flush=True)
 n.destroy_node();rclpy.shutdown();stop_process()
