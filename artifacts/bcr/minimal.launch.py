from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py')),launch_arguments={'gz_args':'-s -r /home/geng/bcr_ws/validation/minimal.sdf'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('bcr_bot'),'launch','bcr_bot_ign_spawn.launch.py')),launch_arguments={'camera_enabled':'false','stereo_camera_enabled':'false','two_d_lidar_enabled':'true','odometry_source':'world'}.items())
    ])
