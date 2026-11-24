#!/usr/bin/env python3
"""Reproduce one saved case with DWB score recording; preserve baseline evidence."""
import os
os.environ.update(ROS_DOMAIN_ID='25',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
from pathlib import Path
import json,subprocess,signal,time,sys,yaml
import rclpy
from dwb_msgs.msg import LocalPlanEvaluation
from nav_msgs.msg import Path as NavPath
root=Path.home()/'nav_ws';case=sys.argv[1] if len(sys.argv)>1 else 'box_astar_0'
variant=sys.argv[2] if len(sys.argv)>2 else 'baseline'
src=root/'artifacts/m7_physical'/case;out=root/'artifacts/m7_tracking'/f'{case}_{variant}';out.mkdir(parents=True,exist_ok=True)
(out/'geometry.json').write_bytes((src/'geometry.json').read_bytes())
cfg=yaml.safe_load((src/'params.yaml').read_text());p=cfg['controller_server']['ros__parameters']['FollowPath']
p.update(publish_evaluation=True,short_circuit_trajectory_evaluation=False)
if variant=='pathalign4':p['PathAlign.scale']=4.
elif variant=='goalalign32':p['GoalAlign.scale']=32.
elif variant=='prune06':p['forward_prune_distance']=.6
elif variant!='baseline':raise ValueError(variant)
(out/'params.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
log=(out/'navigation.log').open('w');triallog=(out/'trial_stdout.log').open('w')
launch=subprocess.Popen(['bash',str(root/'src/my_robot_navigation/scripts/start_navigation.sh'),'rviz:=false','gui:=false',
 'world_file:='+str(src/'world.sdf'),'map:='+str(src/'map.yaml'),'params_file:='+str(out/'params.yaml')],stdout=log,stderr=log,start_new_session=True)
trial=subprocess.Popen(['python3',str(root/'src/my_robot_navigation/scripts/physical_scenario_trial.py'),str(out)],stdout=triallog,stderr=triallog)
rclpy.init();n=rclpy.create_node('dwb_score_diagnostic');scores=[];paths=[];last=-1.;subscription=None
def record(m):
 global last
 stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if stamp-last<.4 or not m.twists:return
 last=stamp
 def pack(t):
  return {'vx':t.traj.velocity.x,'w':t.traj.velocity.theta,'total':t.total,
   'critics':{x.name:float(x.raw_score*x.scale) for x in t.scores}}
 best=m.twists[m.best_index]
 moving=[t for t in m.twists if t.total>=0 and t.traj.velocity.x>=.08]
 scores.append({'stamp':stamp,'best':pack(best),'best_moving':pack(min(moving,key=lambda t:t.total)) if moving else None,
                'valid_count':sum(t.total>=0 for t in m.twists),'count':len(m.twists)})
n.create_subscription(NavPath,'/plan',lambda m:paths.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
 'points':[[p.pose.position.x,p.pose.position.y] for p in m.poses]}),10)
try:
 deadline=time.monotonic()+220
 while trial.poll() is None:
  assert time.monotonic()<deadline,'Diagnostic timeout'
  if subscription is None:
   for topic,types in n.get_topic_names_and_types():
    if 'dwb_msgs/msg/LocalPlanEvaluation' in types:
     subscription=n.create_subscription(LocalPlanEvaluation,topic,record,10);print('Recording',topic,flush=True);break
  rclpy.spin_once(n,timeout_sec=.1)
 assert trial.returncode==0
 assert scores,'No DWB scores recorded'
finally:
 if trial.poll() is None:trial.terminate();trial.wait(timeout=15)
 os.killpg(launch.pid,signal.SIGINT)
 try:launch.wait(timeout=20)
 except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
 (out/'diagnostic.json').write_text(json.dumps({'scores':scores,'paths':paths},indent=2))
 n.destroy_node();rclpy.shutdown();log.close();triallog.close()
print((out/'trial_stdout.log').read_text(),flush=True)
