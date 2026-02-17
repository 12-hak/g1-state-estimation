#!/bin/bash

# Build script for G1 Localization on PC2
# Run from ~/development/state_e

echo "=================================="
echo "Building G1 Localization System"
echo "=================================="

# Ensure we're in the right directory
cd ~/development/state_e

# Clean old build
rm -rf build
mkdir -p build
cd build

# Configure CMake (will auto-detect Unitree SDK)
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
make -j$(nproc)

# Check if build succeeded
if [ $? -eq 0 ]; then
    echo ""
    echo "=================================="
    echo "Build complete!"
    echo "=================================="
    echo "TEMP PATH FIX (Run this first if you get symbol errors):"
    echo "export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:/home/unitree/development/unitree_sdk2/lib:\$LD_LIBRARY_PATH"
    echo ""
    echo "Then run with:"
    echo "./build/g1_localization_node eth0 127.0.0.1"
else
    echo ""
    echo "Build failed! Check errors above."
    exit 1
fi
