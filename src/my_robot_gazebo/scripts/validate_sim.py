#!/usr/bin/env python3
"""Validate a fresh M4 simulation; motion is bounded and zeroed in finally."""
import json
import math
from pathlib import Path
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node('m4_validation')
data = {name: [] for name in ('odom', 'truth', 'scan', 'joints', 'clock')}
for cls, topic, key in [(Odometry, '/odom', 'odom'), (Odometry, '/ground_truth', 'truth'),
                        (LaserScan, '/scan', 'scan'), (JointState, '/joint_states', 'joints'),
                        (Clock, '/clock', 'clock')]:
    node.create_subscription(cls, topic, lambda msg, k=key: data[k].append(msg),
                             qos_profile_sensor_data)
pub = node.create_publisher(Twist, '/cmd_vel', 10)
def spin(seconds, command=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if command is not None:
            pub.publish(command)
        rclpy.spin_once(node, timeout_sec=.05)
def pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return [p.x, p.y, p.z, math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
def stamp(c):
    return c.clock.sec + c.clock.nanosec*1e-9
report = {}
out = Path.home() / 'nav_ws/artifacts/m4'
out.mkdir(parents=True, exist_ok=True)
try:
    deadline = time.monotonic() + 45
    while not all(data.values()) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    assert all(data.values()), 'Missing topics: '+str([k for k,v in data.items() if not v])
    spin(3, Twist())
    before = pose(data['truth'][-1]); clock0 = stamp(data['clock'][-1])
    t0 = time.monotonic(); scan0 = len(data['scan'])
    spin(5, Twist())
    after = pose(data['truth'][-1])
    scan = data['scan'][-1]
    idx = round(-scan.angle_min/scan.angle_increment)
    report['stationary'] = dict(start=before, end=after,
        drift=math.hypot(after[0]-before[0],after[1]-before[1]),
        real_seconds=time.monotonic()-t0, sim_seconds=stamp(data['clock'][-1])-clock0,
        scan_count=len(data['scan'])-scan0, scan_frame=scan.header.frame_id,
        front_range=float(scan.ranges[idx]), beam_count=len(scan.ranges))
    assert report['stationary']['drift'] < .02, 'Stationary drift'
    assert abs(after[2]) < .08, 'Robot ground height incorrect'
    assert .7 < scan.ranges[idx] < 1.1, 'Front box range unexpected'
    for label, velocity, seconds in [('forward',(.1,0.),3),
                                     ('backward',(-.1,0.),3),
                                     ('rotate',(0.,.3),3)]:
        command = Twist(); command.linear.x, command.angular.z = velocity
        start = pose(data['truth'][-1]); odom_start = pose(data['odom'][-1])
        spin(seconds, command); spin(1, Twist())
        end = pose(data['truth'][-1]); odom_end = pose(data['odom'][-1])
        turn = math.atan2(math.sin(end[3]-start[3]), math.cos(end[3]-start[3]))
        report[label] = dict(start=start,end=end,odom_start=odom_start,odom_end=odom_end,
            distance=math.hypot(end[0]-start[0],end[1]-start[1]),yaw_change=turn)
        odom_turn = math.atan2(math.sin(odom_end[3]-odom_start[3]),math.cos(odom_end[3]-odom_start[3]))
        report[label]['odom_yaw_change'] = odom_turn
        report[label]['yaw_difference'] = abs(math.atan2(math.sin(turn-odom_turn),math.cos(turn-odom_turn)))
        print(label, json.dumps(report[label]), flush=True)
        if label == 'rotate':
            assert .5 < turn < 1.2, 'Rotation direction or magnitude incorrect'
            assert report[label]['distance'] < .03, 'Rotation center not aligned'
            assert report[label]['yaw_difference'] < .05, 'Wheel odometry yaw mismatch'
        else:
            assert .18 < report[label]['distance'] < .4, 'Physical translation incorrect'
            assert (end[0]-start[0])*velocity[0] > 0, 'Translation direction reversed'
    start = pose(data['truth'][-1]); spin(5,Twist());end=pose(data['truth'][-1])
    report['stop_drift'] = math.hypot(end[0]-start[0],end[1]-start[1])
    assert report['stop_drift'] < .02, 'Stopping drift'
    report['passed'] = True
except Exception as exc:
    report.update(passed=False,error=str(exc))
    raise
finally:
    spin(.5,Twist())
    (out/'validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)
    node.destroy_node();rclpy.shutdown()
