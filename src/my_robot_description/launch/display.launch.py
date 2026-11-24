from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    share = Path(get_package_share_directory('my_robot_description'))
    description = xacro.process_file(str(share / 'urdf/my_robot.urdf.xacro')).toxml()
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': False}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': False}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz/model.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': False}]),
    ])
