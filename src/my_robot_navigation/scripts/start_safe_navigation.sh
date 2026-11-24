#!/usr/bin/env bash
# Use with an already running M7 A* simulation; does not start another world.
set -e
source /opt/ros/humble/setup.bash
source "$HOME/nav_ws/install/setup.bash"
source "$HOME/tf_fix_ws/install/setup.bash"
export ROS_DOMAIN_ID=25
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
nav_prefix="$(ros2 pkg prefix my_robot_navigation)"
exec python3 "$nav_prefix/lib/my_robot_navigation/safe_navigate.py" "$@"
