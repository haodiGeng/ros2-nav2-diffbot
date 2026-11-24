#!/usr/bin/env bash
# Start one M7 demonstration world; use from an Ubuntu desktop terminal for RViz.
set -e
source /opt/ros/humble/setup.bash
source "$HOME/nav_ws/install/setup.bash"
source "$HOME/tf_fix_ws/install/setup.bash"
nav_prefix="$(ros2 pkg prefix my_robot_navigation)"
demo_dir="$HOME/nav_ws/artifacts/m7_continuous_v2"
if [[ ! -f "$demo_dir/world.sdf" || ! -f "$demo_dir/map.yaml" ]]; then
  python3 "$nav_prefix/lib/my_robot_navigation/continuous_acceptance.py" --prepare-only
fi
exec bash "$nav_prefix/lib/my_robot_navigation/start_astar_navigation.sh" \
  world_file:="$demo_dir/world.sdf" map:="$demo_dir/map.yaml" "$@"
