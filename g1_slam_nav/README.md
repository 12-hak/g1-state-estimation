# G1 SLAM Navigation System

Closed-loop SLAM and navigation stack for the Unitree G1 humanoid robot (and Go2 quadruped), built on ROS2 Humble with C++ performance-critical paths and a React web interface.

## Architecture

```
Livox Mid-360 ──> livox_bridge_node ──> /pointcloud ──> slam_node ──> /slam/map
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
| `g1_sensor_bridge` | C++ | Livox SDK + Unitree SDK2 to ROS2 |
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
sudo apt install ros-humble-desktop ros-humble-nav2-bringup \
  ros-humble-pcl-conversions ros-humble-pcl-ros \
  ros-humble-tf2-eigen python3-websockets
```

### Hardware SDKs
- Unitree SDK2: `/home/unitree/development/unitree_sdk2`
- Livox SDK2: system-installed `livox_lidar_sdk_shared`

### Frontend
```bash
cd src/g1_web_interface/frontend
npm install && npm run build
```

## Configuration

Robot-specific parameters in `g1_bringup/config/`:
- `g1_robot.yaml` - G1 humanoid (29 joints, upside-down LiDAR)
- `go2_robot.yaml` - Go2 quadruped (12 joints, standard LiDAR mount)

SLAM tuning in `g1_slam/config/slam_params.yaml`
Nav2 tuning in `g1_navigation/params/nav2_params.yaml`
