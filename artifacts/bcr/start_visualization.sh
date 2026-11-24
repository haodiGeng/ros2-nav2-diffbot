#!/usr/bin/env bash
# Run in an Ubuntu desktop terminal after starting navigation.
set -e
source /opt/ros/humble/setup.bash
source "$HOME/bcr_ws/install/setup.bash"
source "$HOME/tf_fix_ws/install/setup.bash"
export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LIBGL_ALWAYS_SOFTWARE=1
exec rviz2 -d "$HOME/bcr_ws/validation/navigation_demo.rviz" --ros-args -p use_sim_time:=true
