# G1 SLAM Navigation System

Closed-loop SLAM and navigation stack for the Unitree G1 humanoid robot (and Go2 quadruped), built on ROS2 Humble with C++ performance-critical paths and a React web interface.

## Architecture

```
Livox Mid-360 ──> livox_bridge_node (stabilized/deskewed) ──> /pointcloud ──> slam_node ──> /slam/map
                                   ──> /imu/lidar  ──>            ──> /slam/pose
                                                                   ──> /tf (map->odom)
Unitree SDK2  ──> unitree_bridge_node ──> /joint_states ──> leg_odometry_node ──> /odom/legs
                                      ──> /imu/body
                                                      
slam_node + occupancy_grid_node ──> /map (OccupancyGrid) ──> Nav2 ──> /cmd_vel

web_bridge_node (WebSocket:9090 + HTTP:8080) <──> React Frontend
```

## Packages

| Package | Language | Description |
|---------|----------|-------------|
| `g1_msgs` | IDL | Custom messages and services |
| `g1_sensor_bridge` | C++ | Livox SDK + Unitree SDK2 to ROS2; Livox IMU-based motion compensation (LIO-Livox style) for stabilized `/pointcloud` |
| `g1_slam` | C++ | KISS-ICP odometry, EKF fusion, loop closure, pose graph, occupancy grid |
| `g1_navigation` | C++ | Nav2 configuration + command bridge |
| `g1_web_interface` | Python/TS | WebSocket bridge + React map UI |
| `g1_bringup` | Launch | Master launch files for G1 and Go2 |

## Quick Start

```bash
# Build
cd g1_slam_nav
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source
source install/setup.bash

# Full system (G1)
ros2 launch g1_bringup g1_full.launch.py

# SLAM-only (mapping)
ros2 launch g1_bringup g1_slam_only.launch.py

# Navigation with saved map
ros2 launch g1_bringup g1_nav_only.launch.py map_file:=/home/unitree/maps/office.pcd

# Go2 variant
ros2 launch g1_bringup go2_full.launch.py
```

Web interface: `http://<robot-ip>:8080`

## Dependencies

### System (apt)
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-nav2-bringup ros-humble-nav2-msgs \
  ros-humble-pcl-conversions ros-humble-pcl-ros \
  ros-humble-tf2-eigen python3-websockets
```
If `nav2_msgs` is still not found by CMake, install it explicitly: `sudo apt install ros-humble-nav2-msgs`

**If `ros-humble-pcl-ros` / `ros-humble-pcl-conversions` are "Unable to locate":**

1. **Confirm ROS 2 repo is set up** (Humble needs Ubuntu 22.04):
   ```bash
   sudo apt update && apt-cache search ros-humble-pcl
   ```
   If you see no results, add the ROS 2 repo and update:
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
   ```

2. **If you're not on Humble** (e.g. Jetson with Ubuntu 20.04 / Galactic), use your distro name:
   ```bash
   echo $ROS_DISTRO   # e.g. galactic
   sudo apt install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions
   ```

3. **Build without SLAM for now:** The workspace is set up so `g1_slam` and `g1_navigation` are optional when their dependencies are missing. You can still build and run the sensor bridge and other packages; install PCL packages when your repo has them and rebuild.

### Hardware SDKs
- Unitree SDK2: `/home/unitree/development/unitree_sdk2`
- Livox SDK2: system-installed `livox_lidar_sdk_shared`

### Frontend
```bash
cd src/g1_web_interface/frontend
npm install && npm run build
```

## Configuration

**Stabilized point cloud (Livox Mid-360):** The Livox bridge applies IMU-based motion compensation (deskew) so `/pointcloud` is stabilized for better mapping. Enabled by default. Parameters in the bridge node:
- `use_stabilized` (bool, default `true`) – enable/disable deskew.
- `deskew_span_s` (double, default `0.012`) – assumed time span of each packet in seconds for per-point time assignment.

Robot-specific parameters in `g1_bringup/config/`:
- `g1_robot.yaml` - G1 humanoid (29 joints, upside-down LiDAR)
- `go2_robot.yaml` - Go2 quadruped (12 joints, standard LiDAR mount)

SLAM tuning in `g1_slam/config/slam_params.yaml`
Nav2 tuning in `g1_navigation/params/nav2_params.yaml`

## Running on G1 (Optimized)

To avoid memory issues (`bad_alloc`) and ensure the LiDAR connects correctly, use this exact sequence on the Jetson:

```bash
# 1. Isolate the network and set library paths
export ROS_DOMAIN_ID=42
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:/home/unitree/development/unitree_sdk2/lib:$LD_LIBRARY_PATH

# 2. Source the build
cd ~/development/state_e/g1_slam_nav
source install/setup.bash

# 3. Launch Full Stack
# (Note: Use width/height to bound map memory if needed, e.g. 50.0 for smaller areas)
ros2 launch g1_bringup g1_full.launch.py \
    lidar_config:=/home/unitree/development/state_e/g1_mid360_config.json
```

### Accessing the Interface
Open a browser on your PC (connected to the same network) and go to:
`http://192.168.123.164:8080`

---

## Troubleshooting (Jetson / runtime)

**`unitree_bridge_node: undefined symbol: ddsi_sertype_v0`**  
Ensure you have exported the `LD_LIBRARY_PATH` shown above so the robot's specific libraries are prioritized.

**`bad_alloc caught: std::bad_alloc`**
This is caused by the Jetson running out of RAM. 
1. **Isolate with Domain ID**: Ensure `export ROS_DOMAIN_ID=42` is set in every terminal.
2. **Add Swap**: `sudo fallocate -l 4G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile`

**Livox: "Failed to initialize Livox SDK"**
Ensure `host_ip` in `g1_mid360_config.json` matches the Jetson's IP (`192.168.123.164`).

**Website shows "Directory listing" instead of Map**
This happens if the frontend path is not set correctly. The system will look for the built React app in `install/g1_web_interface/share/g1_web_interface/frontend`. Use the latest `web_bridge_node.py` which auto-detects this path.
