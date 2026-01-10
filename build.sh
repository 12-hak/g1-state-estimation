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
    echo "Run with: ./build/g1_localization_node [network_interface] [receiver_ip]"
    echo "Example: ./build/g1_localization_node eth0 192.168.1.169"
else
    echo ""
    echo "Build failed! Check errors above."
    exit 1
fi
