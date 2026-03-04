#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-foxy}"
G1_WS="${G1_WS:-$HOME/state_e/g1_slam_nav}"
DOMAIN_ID="${ROS_DOMAIN_ID:-10}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${G1_WS}/install/setup.bash"

export ROS_DOMAIN_ID="${DOMAIN_ID}"

echo "[1/4] Required input topics"
ros2 topic info /livox/lidar >/dev/null
ros2 topic info /livox/imu >/dev/null

echo "[2/4] Required Point-LIO outputs"
ros2 topic info /aft_mapped_to_init >/dev/null
ros2 topic info /Laser_map >/dev/null
ros2 topic info /cloud_registered >/dev/null

echo "[3/4] Rate checks (5s sample)"
timeout 7s ros2 topic hz /livox/lidar || true
timeout 7s ros2 topic hz /aft_mapped_to_init || true
timeout 7s ros2 topic hz /Laser_map || true

echo "[4/4] Web bridge endpoints"
ros2 topic list | rg "aft_mapped_to_init|Laser_map|cloud_registered|path|map" || true

echo "PC Point-LIO smoke checks complete."
