# G1 State Estimation & 3D Mapping

Real-time localization and high-fidelity 3D mapping system for the Unitree G1 humanoid robot.

## Features

### 🎯 Real-Time Localization
- **50Hz localization** using leg odometry + LiDAR ICP
- **2D scan matching** with adaptive map building
- **Drift correction** via SLAM fusion
- **Body-filtered scans** (removes robot's own structure)
- **UDP visualization** for real-time monitoring

### 🗺️ High-Fidelity 3D Mapping
- **Standalone recorder** for offline mapping
- **Full-resolution point clouds** (no downsampling during capture)
- **Pose-synchronized scans** for accurate alignment
- **Professional output formats** (PLY/PCD)
- **Interior filtering** to remove clutter

## Architecture

```
┌─────────────────────┐         ┌──────────────────┐
│  g1_localization    │ UDP     │  g1_network_viz  │
│  (Robot)            │────────>│  (PC - Optional) │
│  - Leg Odometry     │ 9870    │  - MuJoCo Viz    │
│  - LiDAR ICP        │         │  - Real-time     │
│  - SLAM Fusion      │         └──────────────────┘
│  - Pose Broadcast   │
└──────────┬──────────┘
           │ UDP 9871
           v
┌─────────────────────┐         ┌──────────────────┐
│  g1_lidar_recorder  │ Files   │  Converter       │
│  (Robot)            │────────>│  (Offline)       │
│  - Raw Scans        │         │  - PLY/PCD       │
│  - Pose Sync        │         │  - Filtering     │
└─────────────────────┘         └──────────────────┘
```

## Quick Start

### Build
```bash
cd ~/development/state_e
mkdir -p build && cd build
cmake ..
make -j4
```

### Run Localization
```bash
# Terminal 1: Localization (required)
./g1_localization_node eth0 127.0.0.1

# Terminal 2: Visualization (optional, on PC)
python g1_network_viz.py
```

### Record 3D Map
```bash
# Terminal 1: Localization
./g1_localization_node eth0 127.0.0.1

# Terminal 2: Recorder
./g1_lidar_recorder ./recordings

# Walk around, then Ctrl+C both
```

### Convert to Point Cloud
```bash
# Basic conversion
python convert_recording_to_pcd.py recordings/recording_20260111_120453

# With interior filtering (recommended)
python convert_recording_to_pcd.py recordings/recording_20260111_120453 --filter-interior

# Output: recording_20260111_120453.ply
```

### View Map
- **CloudCompare**: https://www.cloudcompare.org/ (recommended)
- **MeshLab**: https://www.meshlab.net/
- **Blender**: Import PLY for rendering
- **Online**: https://www.creators3d.com/online-viewer

## Components

### g1_localization_node
Real-time state estimator combining:
- **Leg odometry** from joint velocities
- **LiDAR scan matching** (2D ICP)
- **IMU orientation** for gravity alignment
- **SLAM correction** to reduce drift

**Performance**: 50Hz localization, <5% CPU

### g1_lidar_recorder
Standalone recorder that captures:
- **Raw LiDAR scans** (full resolution)
- **Synchronized poses** from localizer
- **Binary format** for efficiency

**Output**:
- `recording_YYYYMMDD_HHMMSS_poses.txt` - Pose trajectory
- `recording_YYYYMMDD_HHMMSS_clouds.bin` - Point clouds

### convert_recording_to_pcd.py
Post-processing tool:
- Transforms scans to world frame
- Optional interior filtering
- Exports to PLY/PCD formats

**Options**:
- `--format ply|pcd` - Output format (default: ply)
- `--filter-interior` - Remove interior clutter

### g1_network_viz.py
Real-time MuJoCo visualization:
- Robot pose and joints
- LiDAR scan (current)
- 2D localization map
- UDP streaming from robot

## Configuration

### Localization Parameters
Edit `src/G1Localizer.cpp`:
- `step_scale_` - Leg odometry scale (default: 0.65)
- `position_alpha_` - ICP fusion weight (default: 0.3)
- Keyframe distance - Map update threshold (default: 0.2m)

### Body Filtering
Edit `src/G1Localizer.cpp` line ~256:
```cpp
if (dist_xy < 0.3f) continue;  // Body radius
if (p_level.z() > -0.5f && p_level.z() < 0.5f && dist_xy < 0.5f) continue;  // Head supports
```

### Recording Rate
Edit `src/LiDARRecorder.cpp` line 169:
```cpp
std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20Hz
```

## Dependencies

### C++ (Robot)
- **Unitree SDK2** - Robot interface
- **Livox SDK2** - LiDAR interface
- **Eigen3** - Linear algebra
- **CMake** - Build system

### Python (PC/Offline)
- **NumPy** - Point cloud processing
- **MuJoCo** - Visualization (optional)
- **Socket** - UDP communication

## File Structure
```
state_e/
├── src/
│   ├── G1Localizer.{cpp,hpp}      # Main localization
│   ├── LiDARRecorder.{cpp,hpp}    # 3D recorder
│   ├── ScanMatcher.{cpp,hpp}      # 2D ICP
│   ├── LivoxInterface.{cpp,hpp}   # LiDAR driver
│   ├── UDPPublisher.{cpp,hpp}     # Network output
│   ├── g1_localization_node.cpp   # Main entry
│   └── g1_lidar_recorder.cpp      # Recorder entry
├── convert_recording_to_pcd.py    # Offline converter
├── g1_network_viz.py              # Real-time viz
├── test_pose_receiver.py          # Debug tool
└── CMakeLists.txt                 # Build config
```

## Tips & Tricks

### Better Maps
1. **Walk slowly** - Gives ICP time to converge
2. **Revisit areas** - Improves consistency
3. **Use --filter-interior** - Removes clutter
4. **Post-process in CloudCompare** - SOR filter, subsampling

### Debugging
```bash
# Test if poses are broadcasting
python test_pose_receiver.py

# Check recording files
head recordings/recording_*_poses.txt
ls -lh recordings/recording_*_clouds.bin
```

### Performance
- Localization: ~50Hz on G1's Jetson
- Recording: ~20Hz (configurable)
- Recorder has **zero impact** on localization

## Known Limitations
- **Drift**: Long-term drift without loop closure
- **Transparent surfaces**: LiDAR sees through glass
- **Dynamic objects**: Moving objects create artifacts
- **2D localization**: Only uses horizontal plane for ICP

## Future Enhancements
- [ ] Loop closure detection
- [ ] 3D ICP for full 6DOF localization
- [ ] Multi-session map merging
- [ ] Intensity-based coloring
- [ ] ROS2 integration

## License
MIT

## Authors
Unitree G1 State Estimation Team

## Acknowledgments
- Unitree Robotics for SDK
- Livox for LiDAR SDK
- KISS-ICP for inspiration
