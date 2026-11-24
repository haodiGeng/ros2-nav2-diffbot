#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/bcr_ws/install/setup.bash"
source "$HOME/tf_fix_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=23
export IGN_PARTITION=bcr_m2
export LIBGL_ALWAYS_SOFTWARE=1
export IGN_GAZEBO_RESOURCE_PATH="$HOME/bcr_ws/install/bcr_bot/share:$HOME/bcr_ws/install/bcr_bot/share/bcr_bot/models"
exec ros2 launch "$HOME/bcr_ws/validation/navigation_demo.launch.py"
