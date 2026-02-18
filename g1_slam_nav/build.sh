#!/bin/bash
set -e

echo "=== G1 SLAM Nav Build ==="
echo ""

# Check for ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
fi

echo "ROS2 Distro: $ROS_DISTRO"
echo ""

# Build frontend (optional, skip if node not available)
if command -v npm &> /dev/null; then
    echo "--- Building React Frontend ---"
    cd src/g1_web_interface/frontend
    npm install --legacy-peer-deps
    npm run build
    cd ../../..
    echo ""
fi

# Build ROS2 packages
echo "--- Building ROS2 Packages ---"
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-select g1_msgs g1_sensor_bridge g1_slam g1_navigation g1_web_interface g1_bringup \
    --symlink-install

echo ""
echo "=== Build Complete ==="
echo "Source with: source install/setup.bash"
echo "Launch with: ros2 launch g1_bringup g1_full.launch.py"
