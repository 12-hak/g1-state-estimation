#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-foxy}"
G1_WS="${G1_WS:-$HOME/state_e/g1_slam_nav}"
DOMAIN_ID="${ROS_DOMAIN_ID:-10}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${G1_WS}/install/setup.bash"

export ROS_DOMAIN_ID="${DOMAIN_ID}"

exec ros2 launch g1_bringup point_lio_pc_mapper.launch.py "$@"
