#!/bin/bash

# G1 Localization Run Script with Path Fixes
# Forces the use of Unitree-supplied DDS libraries

# 1. Ensure we are in the script's directory
cd "$(dirname "$0")"
export PROJECT_ROOT=$(pwd)

# 2. Set the Library Path
export UNITREE_SDK_LIB="/home/unitree/development/unitree_sdk2/lib"
export CYCLONEDDS_LIB="/opt/unitree_robotics/lib"
export LD_LIBRARY_PATH=$CYCLONEDDS_LIB:$UNITREE_SDK_LIB:$LD_LIBRARY_PATH

# 3. Check for dependencies
if [ ! -f "build/g1_localization_node" ]; then
    echo "Error: g1_localization_node not found. Run ./build.sh first."
    exit 1
fi

if [ ! -f "g1_mid360_config.json" ]; then
    echo "Error: g1_mid360_config.json not found in $PROJECT_ROOT"
    exit 1
fi

# 3. Handle arguments
IFACE=${1:-"eth0"}
IP=${2:-"127.0.0.1"}

echo "--- G1 Localization Node ---"
echo "Interface: $IFACE"
echo "Receiver:  $IP"
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "----------------------------"

# 4. Run the node
./build/g1_localization_node $IFACE $IP
