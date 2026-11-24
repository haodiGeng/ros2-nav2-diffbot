from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    share = Path(get_package_share_directory('my_robot_navigation'))
    slam = Path(get_package_share_directory('slam_toolbox'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share/'config/slam_mapping.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(slam/'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time':'true',
                              'slam_params_file':LaunchConfiguration('params_file')}.items())
    ])
