# SLAM Review and Alternatives for G1

## Why We Struggled

1. **Strict alignment gating** – We only added scans to the map when ICP had high overlap (e.g. 40%+, fitness < 0.015). That kept the map “clean” but made it easy to **lose loc**: once alignment failed, we never added again until the scan overlapped enough, so the blue map stopped growing and the robot stayed in SEEKING.

2. **Translation-only ICP** – We locked rotation to odometry to fix spinning, but that made us fully dependent on odom/IMU yaw. If yaw was wrong or stale, the map and scan could still look wrong.

3. **KISS-ICP is minimal** – It’s a lightweight scan-to-map ICP. It doesn’t do LiDAR–IMU tight coupling or per-point time correction. For “logical map building based on movement and over-scan,” we need either a simpler always-add strategy or a proper LIO stack.

---

## How Others Do It (Unitree / Livox)

### Unitree’s own stack: Point-LIO

- **Repo:** [unitreerobotics/point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar) (ROS1, L1/L2).
- **ROS2 port (works with Livox):** [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2).
- **What it does:** LiDAR–inertial odometry (LIO): fuses **per-point LiDAR time** + **IMU** for motion compensation and pose. Builds a consistent point cloud map from motion; robust to vibration and fast motion.
- **Catch:** Prefers **Livox CustomMsg** (per-point timestamps). Our current bridge publishes `PointCloud2` without per-point time; Point-LIO can still work with external IMU and a single timestamp per scan if configured (see “Livox serials with external IMU” in their README).

### Other options

| Approach | Notes |
|----------|------|
| **Fast-LIO** | Livox-oriented LIO; GPL-2.0. |
| **open3d_slam** | LiDAR-only graph SLAM; no IMU. |
| **slam_toolbox** | 2D occupancy grid; needs `pointcloud_to_laserscan` for 3D LiDAR. |
| **mp2p_icp** (ROS2) | Multi-primitive ICP; `sudo apt install ros-humble-mp2p-icp`. |
| **CT-ICP** | Continuous-time LiDAR odometry; good for motion. |

---

## What We Can Do in This Repo

### Option A: Simpler “motion + over-scan” mode (no alignment gate)

- **Idea:** Always add the current scan to the map using the **odom-based pose** (and optional light ICP refinement). No “only add when aligned” rule.
- **Pros:** Blue map always grows as the robot moves; no permanent “lost loc”; map is built from movement and over-scan.
- **Cons:** Map can drift or smear if odom is bad; loop closure / post-processing can correct later.
- **Implementation:** A parameter e.g. `map_build_mode: motion` that skips the alignment gate and adds every (or every N-th) scan with the current pose. Keep `map_build_mode: strict` for the current behavior.

### Option B: Try Point-LIO ROS2 as the front-end

- Use **Point-LIO ROS2** to produce **odometry + map** from Livox + IMU.
- Our stack then:
  - Subscribes to Point-LIO’s odom and map (or point cloud).
  - Uses that for Nav2 and the web UI (no KISS-ICP in the loop).
- Requires: Livox driver that can publish what Point-LIO expects (CustomMsg or compatible PointCloud2 + IMU), and correct IMU/LiDAR extrinsics and config (see Point-LIO README).

### Option C: Keep KISS-ICP but relax and simplify

- Lower the bar for “aligned” (e.g. 25% overlap, fitness < 0.05) so we add more often and lose loc less.
- Optionally run **full 6-DOF ICP** again (not translation-only) with a strong odom prior and outlier rejection, so we get small yaw corrections from the scan instead of relying only on odom.

---

## Recommendation

1. **Short term:** Add **Option A** (`map_build_mode: motion`) so you get a **solid blue map from movement and over-scan** without depending on strict alignment. You can still show “aligned” vs “seeking” for UX, but the map keeps building.
2. **Medium term:** Try **Point-LIO ROS2** with the G1’s Livox Mid-360 + IMU (and Unitree SDK if needed) for robust localization and logical map building the way Unitree and others do it; then plug that into this repo’s nav and web UI.
3. **Tuning:** If you stay on KISS-ICP, use `motion` mode to build the map, and consider a separate “localization-only” mode (e.g. load a prior map and only run ICP for pose, no map update) for repeat runs.

---

## References

- Point-LIO ROS2 (Livox + Unitree L1/L2): https://github.com/dfloreaa/point_lio_ros2  
- Unitree Point-LIO (ROS1): https://github.com/unitreerobotics/point_lio_unilidar  
- Livox ROS2 driver: https://github.com/Livox-SDK/livox_ros_driver2  
- SLAM Toolbox (2D): https://github.com/SteveMacenski/slam_toolbox  
- mp2p_icp (ROS2): `ros-humble-mp2p-icp`
