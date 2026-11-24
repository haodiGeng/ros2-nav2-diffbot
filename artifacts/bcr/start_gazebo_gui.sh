#!/usr/bin/env bash
# Run in an Ubuntu desktop terminal after start_navigation.sh.
set -e
source /opt/ros/humble/setup.bash
source "$HOME/bcr_ws/install/setup.bash"
export IGN_PARTITION=bcr_m2
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
export IGN_GAZEBO_RESOURCE_PATH="$HOME/bcr_ws/install/bcr_bot/share:$HOME/bcr_ws/install/bcr_bot/share/bcr_bot/models"
exec ign gazebo -g --force-version 6
