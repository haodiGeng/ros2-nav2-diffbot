import json
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node('bcr_avoidance_trial')
node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
odometry = []
plans = []
node.create_subscription(Odometry, '/bcr_bot/odom', odometry.append, qos_profile_sensor_data)
node.create_subscription(Path, '/plan', plans.append, 10)
publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

def spin_for(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

def wait(future, seconds):
    deadline = time.monotonic() + seconds
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.done()

spin_for(5)
assert odometry, 'No odometry'
initial = PoseWithCovarianceStamped()
initial.header.frame_id = 'map'
initial.pose.pose = odometry[-1].pose.pose
initial.pose.covariance[0] = 0.05
initial.pose.covariance[7] = 0.05
initial.pose.covariance[35] = 0.05
for _ in range(3):
    publisher.publish(initial)
    spin_for(1)
spin_for(5)
client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
assert client.wait_for_server(timeout_sec=30), 'No navigation server'
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = 'map'
goal.pose.pose.orientation.w = 1.0
future = client.send_goal_async(goal)
assert wait(future, 20), 'Goal response timeout'
handle = future.result()
assert handle.accepted, 'Goal rejected'
print('Goal accepted', flush=True)
start = time.monotonic()
future = handle.get_result_async()
if not wait(future, 180):
    cancellation = handle.cancel_goal_async()
    wait(cancellation, 15)
    wait(future, 15)
result = {
    'status': future.result().status if future.done() else 'timeout',
    'wall_seconds': time.monotonic() - start,
    'goal': [0, 0],
    'obstacle_center': [1.2, 0],
    'obstacle_size': [0.4, 0.4, 0.5],
    'odometry': [[m.pose.pose.position.x, m.pose.pose.position.y] for m in odometry],
    'plans': [[[p.pose.position.x, p.pose.position.y] for p in m.poses] for m in plans],
}
with open('/home/geng/bcr_ws/validation/avoidance_result.json', 'w') as output:
    json.dump(result, output)
print('Result', result['status'], 'seconds', result['wall_seconds'], flush=True)
node.destroy_node()
rclpy.shutdown()
