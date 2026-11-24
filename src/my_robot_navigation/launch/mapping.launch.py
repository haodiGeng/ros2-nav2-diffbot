from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav = Path(get_package_share_directory('my_robot_navigation'))
    sim = Path(get_package_share_directory('my_robot_gazebo'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim/'launch/sim.launch.py')),
                                launch_arguments={'gui':LaunchConfiguration('gui')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav/'launch/slam.launch.py'))),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d',str(nav/'rviz/mapping.rviz')],
             parameters=[{'use_sim_time':True}],
             condition=IfCondition(LaunchConfiguration('rviz')))
    ])
