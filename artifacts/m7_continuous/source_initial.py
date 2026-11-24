#!/usr/bin/env python3
import os
os.environ.update(ROS_DOMAIN_ID='25',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
from pathlib import Path
import json,subprocess,signal,xml.etree.ElementTree as ET,time
import numpy as np
import yaml,rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from safe_navigate import GatedNavigator
r=Path.home()/'nav_ws';out=r/'artifacts/m7_continuous';out.mkdir(parents=True,exist_ok=True)
tree=ET.parse(r/'src/my_robot_gazebo/worlds/training.sdf');w=tree.getroot().find('world')
for m in list(w.findall('model')):
 if m.get('name')!='ground':w.remove(m)
boxes=[(-4.075,0,.15,8.3),(4.075,0,.15,8.3),(0,4.075,8.3,.15),(0,-4.075,8.3,.15),(0,1.2,.4,.4)]
for x,gap in [(-2.,.8),(2.,1.4)]:
 for sign in [-1,1]:boxes.append((x,sign*(4+gap/2)/2,1.,4-gap/2))
for i,(x,y,sx,sy) in enumerate(boxes):
 m=ET.SubElement(w,'model',name=f'obstacle_{i}');ET.SubElement(m,'static').text='true';ET.SubElement(m,'pose').text=f'{x} {y} .5 0 0 0';link=ET.SubElement(m,'link',name='body')
 for kind in ['visual','collision']:
  box=ET.SubElement(ET.SubElement(ET.SubElement(link,kind,name=kind),'geometry'),'box');ET.SubElement(box,'size').text=f'{sx} {sy} 1'
tree.write(out/'world.sdf',encoding='unicode')
yy,xx=np.mgrid[:168,:168];x=-4.2+(xx+.5)*.05;y=-4.2+(yy+.5)*.05;occupied=np.zeros((168,168),bool)
for cx,cy,sx,sy in boxes:occupied|=(abs(x-cx)<sx/2)&(abs(y-cy)<sy/2)
(out/'map.pgm').write_bytes(b'P5\n168 168\n255\n'+np.flipud(np.where(occupied,0,254).astype('uint8')).tobytes())
(out/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[-4.2,-4.2,0.],negate=0,occupied_thresh=.65,free_thresh=.25)))
(out/'geometry.json').write_text(json.dumps(boxes))
log=(out/'navigation.log').open('w');p=subprocess.Popen(['bash',str(r/'src/my_robot_navigation/scripts/start_astar_navigation.sh'),'rviz:=false','gui:=false','map:='+str(out/'map.yaml'),'world_file:='+str(out/'world.sdf')],stdout=log,stderr=log,start_new_session=True)
rclpy.init();n=rclpy.create_node('continuous_acceptance');n.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
truth=[];n.create_subscription(Odometry,'/ground_truth',lambda m:truth.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.pose.position.x,m.pose.pose.position.y]),10)
rows=[]
try:
 for name in ['amcl','planner_server','controller_server','bt_navigator']:
  c=n.create_client(GetState,'/'+name+'/get_state');assert c.wait_for_service(timeout_sec=45);end=time.monotonic()+45
  while True:
   f=c.call_async(GetState.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5)
   if f.done() and f.result().current_state.id==3:break
   assert time.monotonic()<end
 gate=GatedNavigator(n)
 for label,x,y,expected in [('around_box',0.,2.6,4),('return',0.,0.,4),('wide',3.3,0.,4),('return_again',0.,0.,4),('narrow_rejected',-3.3,0.,6),('after_rejection',0.,-.8,4)]:
  goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x=x;goal.pose.position.y=y;goal.pose.orientation.w=1.
  start=time.monotonic();result=gate.execute(goal);rows.append(dict(label=label,goal=[x,y],wall_seconds=time.monotonic()-start,**result))
  (out/'results.json').write_text(json.dumps({'rows':rows,'truth':truth},indent=2));print(rows[-1],flush=True)
  assert result['status']==expected and result['recoveries']==0
  if expected==6:assert not result['navigation_goal_sent']
finally:
 (out/'results.json').write_text(json.dumps({'rows':rows,'truth':truth},indent=2));n.destroy_node();rclpy.shutdown();os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=20)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
 log.close()
