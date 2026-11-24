#!/usr/bin/env python3
"""Integration check of installed CLI refusal, success and active-mission guard."""
import os
os.environ.update(ROS_DOMAIN_ID='25',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
from pathlib import Path
import json,signal,subprocess,time
import rclpy
from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import GetState
root=Path.home()/'nav_ws';out=root/'artifacts/m7_gate_entry';out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('gate_entry_validation');active=False
def status(m):
    global active
    active=any(s.status in (1,2,3) for s in m.status_list)
node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',status,10)
rows=[]
try:
    for case in ['gap080_gate_0','gap140_gate_0']:
        src=root/'artifacts/m7_gate_validation'/case;log=(out/(case+'.log')).open('w')
        launch=subprocess.Popen(['bash',str(root/'src/my_robot_navigation/scripts/start_astar_navigation.sh'),'gui:=false','rviz:=false',
            'world_file:='+str(src/'world.sdf'),'map:='+str(src/'map.yaml'),'params_file:='+str(src/'params.yaml')],stdout=log,stderr=log,start_new_session=True)
        cli=None
        try:
            c=node.create_client(GetState,'/bt_navigator/get_state');assert c.wait_for_service(timeout_sec=45)
            deadline=time.monotonic()+45
            while True:
                f=c.call_async(GetState.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=5)
                if f.done() and f.result().current_state.id==3:break
                assert time.monotonic()<deadline
            node.destroy_client(c)
            cli=subprocess.Popen(['bash',str(root/'src/my_robot_navigation/scripts/start_safe_navigation.sh'),'2.4','0'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            if '140' in case:
                active=False;end=time.monotonic()+25
                while not active and cli.poll() is None and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
                assert active,'No executing navigation goal observed'
                second=subprocess.run(['bash',str(root/'src/my_robot_navigation/scripts/start_safe_navigation.sh'),'0','0'],capture_output=True,text=True,timeout=20)
                (out/'active_guard_stderr.txt').write_text(second.stderr)
                assert second.returncode!=0 and 'Another navigation goal is active' in second.stderr
            stdout,stderr=cli.communicate(timeout=180)
            (out/(case+'_stdout.txt')).write_text(stdout);(out/(case+'_stderr.txt')).write_text(stderr)
            assert stdout.strip(),(cli.returncode,stderr)
            result=json.loads(stdout.strip().splitlines()[-1]);expected=2 if '080' in case else 0
            assert cli.returncode==expected,(cli.returncode,stderr)
            assert result['navigation_goal_sent']==('140' in case)
            rows.append({'case':case,'exit_code':cli.returncode,'result':result})
        finally:
            if cli and cli.poll() is None:cli.terminate();cli.wait(timeout=15)
            os.killpg(launch.pid,signal.SIGINT)
            try:launch.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
            log.close()
    (out/'results.json').write_text(json.dumps({'passed':True,'active_goal_refused_without_takeover':True,'cases':rows},indent=2))
    print('Gate entry integration checks passed',flush=True)
finally:node.destroy_node();rclpy.shutdown()
