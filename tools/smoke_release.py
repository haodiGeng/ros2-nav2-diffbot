#!/usr/bin/env python3
"""Drive one isolated release-check simulation, then stop its own process group."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--workspace',type=Path,required=True)
 parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
 workspace=args.workspace.resolve();args.output.parent.mkdir(parents=True,exist_ok=True)
 os.environ.update(ROS_DOMAIN_ID='29',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',MY_ROBOT_DRIVE_TRACK_SCALE='0.995')
 from ament_index_python.packages import get_package_prefix,get_package_share_directory
 prefixes={p:get_package_prefix(p) for p in ['my_planner','my_robot_description','my_robot_gazebo','my_robot_navigation']}
 assert all(Path(v).is_relative_to(workspace/'install') for v in prefixes.values()),prefixes
 nav=Path(get_package_share_directory('my_robot_navigation'))
 scene=workspace/'artifacts/m7_continuous_v2'
 for f in ['map.yaml','world.sdf']:assert (scene/f).is_file()
 report={'completed':False,'package_prefixes':prefixes,'clock':[],'scan_messages':0,'truth':[]}
 log=args.output.with_suffix('.log').open('w')
 process=subprocess.Popen(['ros2','launch','my_robot_navigation','nav_demo.launch.py','gui:=false','rviz:=false',
  'map:='+str(scene/'map.yaml'),'world_file:='+str(scene/'world.sdf'),
  'params_file:='+str(nav/'config/nav2_amcl_sim_calibrated.yaml')],stdout=log,stderr=log,start_new_session=True)
 n=None
 try:
  import rclpy
  from lifecycle_msgs.srv import GetState
  from sensor_msgs.msg import LaserScan
  from nav_msgs.msg import Odometry
  from geometry_msgs.msg import PoseStamped
  from rosgraph_msgs.msg import Clock
  from rclpy.qos import qos_profile_sensor_data
  sys.path.insert(0,str(Path(prefixes['my_robot_navigation'])/'lib/my_robot_navigation'))
  from safe_navigate import GatedNavigator
  rclpy.init();n=rclpy.create_node('m8_release_smoke');n.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
  n.create_subscription(Clock,'/clock',lambda m:report['clock'].append(m.clock.sec+m.clock.nanosec*1e-9),10)
  def scan(m):report['scan_messages']+=1
  n.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
  n.create_subscription(Odometry,'/ground_truth',lambda m:report['truth'].append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.pose.position.x,m.pose.pose.position.y]),10)
  report['lifecycle']={}
  for name in ['amcl','planner_server','controller_server','bt_navigator']:
   c=n.create_client(GetState,'/'+name+'/get_state');assert c.wait_for_service(timeout_sec=45)
   end=time.monotonic()+45
   while True:
    f=c.call_async(GetState.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5)
    if f.done() and f.result().current_state.id==3:report['lifecycle'][name]='active';break
    assert time.monotonic()<end
  goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x=0.;goal.pose.position.y=2.6;goal.pose.orientation.w=1.
  report['navigation']=GatedNavigator(n).execute(goal)
  assert report['navigation']['status']==4 and report['navigation']['recoveries']==0
  assert report['scan_messages']>10 and len(report['clock'])>10 and report['clock'][-1]>report['clock'][0]
  assert report['truth'] and report['truth'][-1][2]>2.3
  report['loaded_libraries']={}
  for path in Path('/proc').glob('[0-9]*/cmdline'):
   try:
    pid=int(path.parent.name)
    if os.getpgid(pid)!=process.pid:continue
    cmd=path.read_bytes().decode(errors='replace')
    if '/nav2_amcl/amcl' not in cmd and '/nav2_planner/planner_server' not in cmd:continue
    libs=sorted({line.split()[-1] for line in (path.parent/'maps').read_text().splitlines()
                 if 'libtf2.so' in line or 'libmy_astar_planner.so' in line})
    report['loaded_libraries']['amcl' if '/nav2_amcl/amcl' in cmd else 'planner_server']=libs
   except (PermissionError,ProcessLookupError,FileNotFoundError):continue
  assert any(str(workspace/'install/my_planner') in x and 'libmy_astar_planner.so' in x for x in report['loaded_libraries'].get('planner_server',[]))
  assert any(str(Path.home()/'tf_fix_ws/install/tf2') in x and 'libtf2.so' in x for x in report['loaded_libraries'].get('amcl',[]))
  report['completed']=True
 except Exception as exc:
  report['error']=repr(exc);raise
 finally:
  if n is not None:n.destroy_node();rclpy.shutdown()
  try:os.killpg(process.pid,signal.SIGINT)
  except ProcessLookupError:pass
  try:process.wait(timeout=20)
  except subprocess.TimeoutExpired:
   os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
  log.close();report['launch_process_exited']=process.poll() is not None
  args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
 print(json.dumps({'completed':report['completed'],'navigation':report.get('navigation'),'output':str(args.output)}))

if __name__=='__main__':main()
