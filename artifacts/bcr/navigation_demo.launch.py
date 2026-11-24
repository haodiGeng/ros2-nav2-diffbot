from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
def generate_launch_description():
    root = Path.home() / 'bcr_ws' / 'validation'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root / 'minimal.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('nav2_bringup')) / 'launch' / 'bringup_launch.py')),
            launch_arguments={'use_sim_time': 'True', 'use_composition': 'False',
                'map': str(root / 'maps' / 'bcr_minimal.yaml'),
                'params_file': str(root / 'nav2_demo.yaml')}.items()),
        ExecuteProcess(cmd=['python3', str(Path.home() / 'bcr_ws' / 'src' / 'bcr_bot' / 'scripts' / 'remapper.py')])
    ])
