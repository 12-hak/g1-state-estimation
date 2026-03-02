#!/bin/bash
# Start Point-LIO stack: Livox driver in background, then Point-LIO mapping in foreground.
# Jetson: ROS2 Foxy. Optional: ROS_DISTRO=foxy WS_LIVOX_INSTALL=... G1_SLAM_NAV_INSTALL=...

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-foxy}"
WS_LIVOX="${WS_LIVOX_INSTALL:-$HOME/ws_livox/install}"
G1_INSTALL="${G1_SLAM_NAV_INSTALL:-$SCRIPT_DIR/../install}"

source /opt/ros/"$ROS_DISTRO"/setup.bash
[ -f "$WS_LIVOX/setup.bash" ] && source "$WS_LIVOX/setup.bash"
[ -f "$G1_INSTALL/setup.bash" ] && source "$G1_INSTALL/setup.bash"

LIVOX_PID=""
cleanup() {
  [ -n "$LIVOX_PID" ] && kill "$LIVOX_PID" 2>/dev/null || true
  exit 0
}
trap cleanup SIGINT SIGTERM

echo "Starting Livox driver in background..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
LIVOX_PID=$!
sleep 5
echo "Starting Point-LIO mapping (Ctrl+C stops both)..."
ros2 launch g1_bringup point_lio_mapping.launch.py "$@" || true
kill "$LIVOX_PID" 2>/dev/null || true
