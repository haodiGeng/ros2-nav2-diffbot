#!/usr/bin/env bash
# Fresh own-robot simulation + AMCL/Nav2; stop any M4/M5 instance first.
set -e
source /opt/ros/humble/setup.bash
source "$HOME/nav_ws/install/setup.bash"
source "$HOME/tf_fix_ws/install/setup.bash"
export ROS_DOMAIN_ID=25
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export IGN_PARTITION=my_robot_m4
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
exec ros2 launch my_robot_navigation nav_demo.launch.py "$@"
