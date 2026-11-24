"""Run sequential navigation goals without resetting AMCL between goals."""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node('navigation_regression')
node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
samples = []
localization = []
results = []
output = Path('/home/geng/bcr_ws/validation') / (
    sys.argv[1] if len(sys.argv) > 1 else 'navigation_regression.json')

def record(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    samples.append([p.x, p.y, yaw, stamp])

node.create_subscription(Odometry, '/bcr_bot/odom', record, qos_profile_sensor_data)

def record_localization(msg):
    if samples:
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        closest = min(samples[-30:], key=lambda s: abs(s[3] - stamp))
        if abs(closest[3] - stamp) <= 0.2:
            localization.append([p.x, p.y, closest[0], closest[1],
                                 stamp, closest[3]])

node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                         record_localization, qos_profile_sensor_data)
client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
if not client.wait_for_server(timeout_sec=30):
    raise RuntimeError('Navigation action unavailable')
deadline = time.monotonic() + 20
while not samples and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
if not samples:
    raise RuntimeError('No odometry received')
for x, y in [(2.2, 0.0), (0.0, 0.0), (2.2, 0.0), (0.0, 1.4)]:
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.w = 1.0
    begin = len(samples)
    pose_begin = len(localization)
    recovery_counts = []
    future = client.send_goal_async(
        goal, feedback_callback=lambda msg: recovery_counts.append(
            msg.feedback.number_of_recoveries))
    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
    if not future.done() or not future.result().accepted:
        raise RuntimeError('Goal not accepted')
    handle = future.result()
    start = time.monotonic()
    future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=150)
    if not future.done():
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=10)
        raise RuntimeError('Goal timed out; cancellation requested')
    result = dict(goal=[x, y], status=future.result().status,
                  wall_seconds=time.monotonic() - start,
                  odometry=samples[begin:], localization=localization[pose_begin:])
    result['recoveries'] = max(recovery_counts, default=0)
    errors = [math.hypot(p[0]-p[2], p[1]-p[3])
              for p in result['localization']]
    result['max_localization_position_error'] = max(errors) if errors else None
    results.append(result)
    output.write_text(json.dumps(results))
    print({k: v for k, v in result.items()
           if k not in ('odometry', 'localization')}, flush=True)
    if result['status'] != 4:
        break
node.destroy_node()
rclpy.shutdown()
