#!/usr/bin/env python3
"""Single-caller, idle-robot preflight gate. Not an emergency stop or runtime arbiter."""
import json,math,sys,time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose,NavigateToPose
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

class GatedNavigator:
    def __init__(self,node):
        self.node=node;self.odom=None;self.active=False
        self.plan=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        self.nav=ActionClient(node,NavigateToPose,'/navigate_to_pose')
        def odom(m):self.odom=(time.monotonic(),m.twist.twist)
        def status(m):self.active=any(x.status in (1,2,3) for x in m.status_list)
        self.subs=[node.create_subscription(Odometry,'/odom',odom,10),node.create_subscription(GoalStatusArray,
            '/navigate_to_pose/_action/status',status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]

    def wait(self,f,seconds):
        rclpy.spin_until_future_complete(self.node,f,timeout_sec=seconds)
        if not f.done() or f.exception():raise RuntimeError('Action request timeout or exception')
        return f.result()

    def require_idle(self):
        if self.active:raise RuntimeError('Another navigation goal is active; refusing takeover')
        if self.odom is None or time.monotonic()-self.odom[0]>1.:raise RuntimeError('No fresh odometry')
        twist=self.odom[1]
        if math.hypot(twist.linear.x,twist.linear.y)>.01 or abs(twist.angular.z)>.01:
            raise RuntimeError('Robot is moving; refusing new mission')

    def execute(self,goal,planner_id='ClearanceGate'):
        if goal.header.frame_id!='map':raise ValueError('Goal must be in map frame')
        if not all(math.isfinite(x) for x in [goal.pose.position.x,goal.pose.position.y]):raise ValueError('Nonfinite goal')
        if not self.plan.wait_for_server(timeout_sec=30) or not self.nav.wait_for_server(timeout_sec=30):
            raise RuntimeError('Navigation actions unavailable')
        end=time.monotonic()+1.5
        while time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.05)
        self.require_idle()
        q=ComputePathToPose.Goal();q.goal=goal;q.planner_id=planner_id;q.use_start=False
        ph=self.wait(self.plan.send_goal_async(q),15)
        result={'planner_id':planner_id,'navigation_goal_sent':False,'preflight_accepted':ph.accepted}
        if not ph.accepted:return {**result,'status':6,'path_points':0,'recoveries':0,'reason':'preflight rejected'}
        pf=ph.get_result_async()
        try:response=self.wait(pf,20)
        except Exception:
            self.wait(ph.cancel_goal_async(),5);raise
        result.update(preflight_status=response.status,path_points=len(response.result.path.poses))
        if response.status!=4 or not response.result.path.poses:
            return {**result,'status':6,'recoveries':0,'reason':'No clearance-compliant plan; navigation not sent'}
        self.require_idle()
        request=NavigateToPose.Goal();request.pose=goal;recoveries=[];handle=None;pending=None
        if planner_id!='ClearanceGate':raise ValueError('Dedicated behavior tree requires ClearanceGate')
        request.behavior_tree=str(Path(get_package_share_directory('my_robot_navigation'))/'behavior_trees/clearance_gate.xml')
        result['behavior_tree']=request.behavior_tree
        try:
            result['navigation_goal_sent']=True
            handle=self.wait(self.nav.send_goal_async(request,feedback_callback=lambda m:recoveries.append(m.feedback.number_of_recoveries)),15)
            if not handle.accepted:return {**result,'status':6,'recoveries':0,'reason':'navigation rejected'}
            result['navigation_goal_sent']=True;pending=handle.get_result_async();response=self.wait(pending,150)
            return {**result,'status':response.status,'recoveries':max(recoveries,default=0),
                'reason':'navigation succeeded' if response.status==4 else 'navigation ended without success'}
        finally:
            if handle and pending and not pending.done():self.wait(handle.cancel_goal_async(),10)

def main():
    if len(sys.argv) not in (3,4):raise SystemExit('Usage: safe_navigate.py X Y [YAW_RADIANS]')
    rclpy.init();n=rclpy.create_node('clearance_gated_navigation')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    try:
        goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x=float(sys.argv[1]);goal.pose.position.y=float(sys.argv[2])
        yaw=float(sys.argv[3]) if len(sys.argv)==4 else 0.
        goal.pose.orientation.z=math.sin(yaw/2);goal.pose.orientation.w=math.cos(yaw/2)
        result=GatedNavigator(n).execute(goal)
        print(json.dumps(result),flush=True)
        if result['status']!=4:raise SystemExit(1 if result['navigation_goal_sent'] else 2)
    finally:n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
