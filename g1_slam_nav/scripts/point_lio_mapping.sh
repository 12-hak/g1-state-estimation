#!/bin/bash
# Start Point-LIO mapping. Run after point_lio_livox.sh (terminal 2).
# Jetson: ROS2 Foxy. Set G1_SLAM_NAV_INSTALL if install is not in ../install relative to this script.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-foxy}"
WS_LIVOX="${WS_LIVOX_INSTALL:-$HOME/ws_livox/install}"
G1_INSTALL="${G1_SLAM_NAV_INSTALL:-$SCRIPT_DIR/../install}"

source /opt/ros/"$ROS_DISTRO"/setup.bash
[ -f "$WS_LIVOX/setup.bash" ] && source "$WS_LIVOX/setup.bash"
[ -f "$G1_INSTALL/setup.bash" ] && source "$G1_INSTALL/setup.bash"

echo "Starting Point-LIO mapping..."
exec ros2 launch g1_bringup point_lio_mapping.launch.py "$@"
