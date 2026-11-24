from pathlib import Path
import os
import math
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = Path(get_package_share_directory('my_robot_gazebo'))
    description_share = Path(get_package_share_directory('my_robot_description'))
    track_scale = float(os.environ.get('MY_ROBOT_DRIVE_TRACK_SCALE', '1.0'))
    if not math.isfinite(track_scale) or track_scale <= 0:
        raise ValueError('MY_ROBOT_DRIVE_TRACK_SCALE must be finite and positive')
    robot = xacro.process_file(str(description_share / 'urdf/my_robot.urdf.xacro'),
                               mappings={'simulation': 'true', 'drive_track_scale': str(track_scale)}).toxml()
    bridges = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        '/model/my_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/model/my_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/model/my_robot/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/model/my_robot/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/world/m4/model/my_robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
    ]
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('world_file', default_value=str(share / 'worlds/training.sdf')),
        SetEnvironmentVariable('IGN_PARTITION', 'my_robot_m4'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', LaunchConfiguration('world_file'),
                            '--force-version', '6'], output='screen'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-g', '--force-version', '6'],
                       condition=IfCondition(LaunchConfiguration('gui')), output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridges,
             parameters=[{'use_sim_time': True}],
             remappings=[('/model/my_robot/cmd_vel', '/cmd_vel'),
                         ('/model/my_robot/odometry', '/odom'),
                         ('/model/my_robot/tf', '/tf'),
                         ('/model/my_robot/scan', '/scan'),
                         ('/model/my_robot/ground_truth', '/ground_truth'),
                         ('/world/m4/model/my_robot/joint_state', '/joint_states')]),
        TimerAction(period=3.0, actions=[
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', 'm4', '-name', 'my_robot',
                            '-string', robot, '-z', '0.02'], output='screen')
        ]),
    ])
