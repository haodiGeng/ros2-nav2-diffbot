#!/usr/bin/env bash
# Opt-in calibration for the current simulation model and analytical demo map.
set -e
source /opt/ros/humble/setup.bash
source "$HOME/nav_ws/install/setup.bash"
source "$HOME/tf_fix_ws/install/setup.bash"
export MY_ROBOT_DRIVE_TRACK_SCALE=0.995
nav_prefix="$(ros2 pkg prefix my_robot_navigation)"
nav_share="$(ros2 pkg prefix --share my_robot_navigation)"
exec bash "$nav_prefix/lib/my_robot_navigation/start_m7_demo.sh" \
  params_file:="$nav_share/config/nav2_amcl_sim_calibrated.yaml" "$@"
