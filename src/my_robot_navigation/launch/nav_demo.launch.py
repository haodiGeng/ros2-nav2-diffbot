from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav=Path(get_package_share_directory('my_robot_navigation'))
    sim=Path(get_package_share_directory('my_robot_gazebo'))
    bringup=Path(get_package_share_directory('nav2_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='false'),
        DeclareLaunchArgument('world_file',default_value=str(sim/'worlds/training.sdf')),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('map',default_value=str(nav/'maps/my_robot_training.yaml')),
        DeclareLaunchArgument('params_file',default_value=str(nav/'config/nav2_params.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim/'launch/sim.launch.py')),
                                 launch_arguments={'gui':LaunchConfiguration('gui'),
                                     'world_file':LaunchConfiguration('world_file')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'launch/bringup_launch.py')),
                                 launch_arguments={'map':LaunchConfiguration('map'),
                                     'params_file':LaunchConfiguration('params_file'),
                                     'use_sim_time':'True','use_composition':'False',
                                     'autostart':'True'}.items()),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(nav/'rviz/navigation.rviz')],
             parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('rviz')))
    ])
