#!/usr/bin/env python3
"""Low-speed scripted sweep for the fixed M4 world, not an autonomous planner."""
import json
import math
from pathlib import Path
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
n=rclpy.create_node('mapping_sweep')
state={}
trace=[]
n.create_subscription(Odometry,'/odom',lambda m:state.update(odom=m,odom_time=time.monotonic()),qos_profile_sensor_data)
n.create_subscription(LaserScan,'/scan',lambda m:state.update(scan=m,scan_time=time.monotonic()),qos_profile_sensor_data)
pub=n.create_publisher(Twist,'/cmd_vel',10)
out=Path.home()/'nav_ws/artifacts/m5'
out.mkdir(parents=True,exist_ok=True)
report={'waypoints':[],'passed':False}
def pose():
    p=state['odom'].pose.pose
    q=p.orientation
    return p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def angle(a):return math.atan2(math.sin(a),math.cos(a))
def step(v,w):
    rclpy.spin_once(n,timeout_sec=.05)
    now=time.monotonic()
    assert all(k in state for k in ('odom','scan')), 'Missing sensor input'
    assert now-state['odom_time']<1.5 and now-state['scan_time']<1.5,'Sensor stream stalled'
    scan=state['scan']
    ranges=[r for i,r in enumerate(scan.ranges)
            if abs(scan.angle_min+i*scan.angle_increment)<.35 and math.isfinite(r)]
    if v>0 and ranges and min(ranges)<.42:
        raise RuntimeError('Front obstacle clearance below 0.42 m')
    cmd=Twist();cmd.linear.x=float(v);cmd.angular.z=float(w);pub.publish(cmd)
    trace.append([now,*pose(),v,w])
def stop():
    end=time.monotonic()+.6
    while time.monotonic()<end:
        pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.05)
try:
    deadline=time.monotonic()+30
    while not all(k in state for k in ('odom','scan')) and time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.1)
    assert 'odom' in state and 'scan' in state,'No sensor data'
    assert math.hypot(*pose()[:2])<.2,'Start from fresh simulation origin'
    for gx,gy in [(-1.8,0.),(-1.8,-1.8),(1.8,-1.8),(1.8,1.8),(-1.8,1.8),(0.,1.8),(0.,0.)]:
        start=time.monotonic()
        while True:
            x,y,yaw=pose()
            dist=math.hypot(gx-x,gy-y)
            if dist<.07:break
            assert time.monotonic()-start<100,'Waypoint timed out'
            err=angle(math.atan2(gy-y,gx-x)-yaw)
            v=min(.15,.6*dist) if abs(err)<.35 else 0.
            step(v,max(-.4,min(.4,1.3*err)))
        stop()
        item={'goal':[gx,gy],'odom':list(pose()),'wall_seconds':time.monotonic()-start}
        report['waypoints'].append(item)
        print(json.dumps(item),flush=True)
        (out/'mapping_sweep.json').write_text(json.dumps(report,indent=2))
    start=time.monotonic()
    while abs(angle(-pose()[2]))>.04:
        assert time.monotonic()-start<20,'Final heading timeout'
        step(0.,max(-.35,min(.35,1.2*angle(-pose()[2]))))
    report['passed']=True
except Exception as e:
    report['error']=str(e)
    raise
finally:
    stop()
    report['trace']=trace
    (out/'mapping_sweep.json').write_text(json.dumps(report))
    n.destroy_node();rclpy.shutdown()
