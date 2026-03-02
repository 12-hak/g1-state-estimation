#!/bin/bash
# Start Livox driver (CustomMsg) for Mid-360. Run this in terminal 1 before point_lio_mapping.sh.
# Jetson: ROS2 Foxy. Set WS_LIVOX_INSTALL if livox is not in ~/ws_livox/install.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-foxy}"
WS_LIVOX="${WS_LIVOX_INSTALL:-$HOME/ws_livox/install}"

source /opt/ros/"$ROS_DISTRO"/setup.bash
[ -f "$WS_LIVOX/setup.bash" ] && source "$WS_LIVOX/setup.bash"

echo "Starting Livox driver (msg_MID360)..."
exec ros2 launch livox_ros_driver2 msg_MID360_launch.py
