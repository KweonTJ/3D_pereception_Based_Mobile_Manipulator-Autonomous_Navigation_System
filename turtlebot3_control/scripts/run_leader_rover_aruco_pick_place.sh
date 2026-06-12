#!/bin/bash
set -eo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/turtlebot3_ws}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

exec ros2 launch turtlebot3_control leader_rover_aruco_pick_place.launch.py "$@"
