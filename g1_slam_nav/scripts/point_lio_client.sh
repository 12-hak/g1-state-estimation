#!/bin/bash
# Single startup on client PC: Nav2 + web map with waypoint/path planning.
# Requires mapping running on Jetson (point_lio_start.sh) and same ROS_DOMAIN_ID.
# Usage: ./point_lio_client.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
G1_INSTALL="${G1_SLAM_NAV_INSTALL:-$SCRIPT_DIR/../install}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
# Use Cyclone DDS so the PC discovers the Jetson (unitree_ros2 uses Cyclone)
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

source /opt/ros/"$ROS_DISTRO"/setup.bash
[ -f "$G1_INSTALL/setup.bash" ] && source "$G1_INSTALL/setup.bash"

echo "Starting Nav2 + web (waypoint/path planning)..."
exec ros2 launch g1_bringup point_lio_navigation.launch.py
