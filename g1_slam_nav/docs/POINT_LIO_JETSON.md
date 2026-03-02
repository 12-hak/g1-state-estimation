# Point-LIO on Jetson (Livox Mid-360, ROS2 Foxy)

This branch adds **Point-LIO** (LiDAR–Inertial Odometry) via the ROS2 port [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2), which works with **Livox** (including Mid-360) when using **livox_ros_driver2** with its **CustomMsg** (per-point timestamps).

**What to copy:** See **[COPY_TO_JETSON.md](COPY_TO_JETSON.md)** for a short list. If you already ran the previous SLAM (KISS-ICP) on the Jetson, **Livox-SDK2** is already there (used by g1_sensor_bridge). You only need to add **livox_ros_driver2** for Point-LIO, then build g1_slam_nav.

---

## 1. Install system dependencies (Foxy)

```bash
sudo apt update
sudo apt install -y ros-foxy-pcl-ros ros-foxy-pcl-conversions ros-foxy-visualization-msgs
sudo apt install -y libeigen3-dev
```

---

## 2. Build Livox-SDK2

**If you already built g1_sensor_bridge (previous KISS-ICP SLAM), Livox-SDK2 is already installed** — the bridge links to `livox_lidar_sdk_shared` and uses `LIVOX_SDK_PATH`. Skip this step.

Otherwise, follow [Livox-SDK2 README](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md). Typical:

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

---

## 3. Build livox_ros_driver2 (separate workspace)

Point-LIO expects **CustomMsg** (per-point timestamps). Use the **msg** launch for Mid-360, not the PointCloud2-only one.

**Foxy:** use `build.sh ROS2`. **Humble:** use `build.sh humble`.

```bash
mkdir -p ~/ws_livox/src
cd ~/ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/ws_livox
source /opt/ros/foxy/setup.bash
./src/livox_ros_driver2/build.sh ROS2
source install/setup.bash
```

Add to `~/.bashrc` so Point-LIO can find the driver and message types:

```bash
source ~/ws_livox/install/setup.bash
```

---

## 4. Copy state_e and build g1_slam_nav (with Point-LIO)

On the Jetson (or after copying the repo):

```bash
cd ~/development   # or your preferred path
git clone <your-state-e-repo-url> state_e
cd state_e
git checkout Point-Lio
git submodule update --init --recursive
```

You should see `g1_slam_nav/src/point_lio_ros2` populated (point_lio package).

Build the **g1_slam_nav** workspace. **Source the Livox driver first** so `livox_ros_driver2` and its message types are available:

```bash
source /opt/ros/foxy/setup.bash
source ~/ws_livox/install/setup.bash
cd /path/to/state_e/g1_slam_nav
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If you get “livox_ros_driver2 not found”, ensure `source ~/ws_livox/install/setup.bash` is run before `colcon build`.

---

## 5. Run Point-LIO with Mid-360

Use the **Livox** driver in **CustomMsg** mode (msg_MID360), then start Point-LIO. Configure the Mid-360 JSON (e.g. `user_config_path`) so the host IP matches the Jetson.

**Option A – Start script (one command, runs Livox in background then Point-LIO):**

```bash
cd /path/to/state_e/g1_slam_nav
chmod +x scripts/*.sh
./scripts/point_lio_start.sh
# Optional: with RViz
./scripts/point_lio_start.sh rviz:=true
```

**Option B – Two terminals:**

**Terminal 1 – Livox driver:**
```bash
cd /path/to/state_e/g1_slam_nav
./scripts/point_lio_livox.sh
```

**Terminal 2 – Point-LIO:**
```bash
cd /path/to/state_e/g1_slam_nav
./scripts/point_lio_mapping.sh
# Optional: ./scripts/point_lio_mapping.sh rviz:=true
```

Scripts use **Foxy** by default and `~/ws_livox/install`; override with:
`ROS_DISTRO=foxy WS_LIVOX_INSTALL=/path/to/ws_livox/install G1_SLAM_NAV_INSTALL=/path/to/g1_slam_nav/install` if needed.

Point-LIO will subscribe to `/livox/lidar` (CustomMsg) and `/livox/imu` and publish odometry and map.

---

## 6. G1-specific notes

- **Config** is in `g1_bringup/config/point_lio_mid360.yaml`. It is set up for **Livox built-in IMU** (topics from livox_ros_driver2). If you want to use the **G1 body IMU** instead, change `imu_topic` to your body IMU topic (e.g. `imu/body`) and set `extrinsic_T` / `extrinsic_R` to the LiDAR pose in the body/IMU frame (we use ~0.1, 0, 0.6 and orientation for the G1 head lidar).
- **IMU sync:** Point-LIO works best when LiDAR and IMU are synchronized. The Livox driver’s built-in IMU is aligned with the LiDAR; body IMU needs correct time offset and extrinsics.
- **Saturation:** If you see IMU saturation warnings, adjust `satu_acc` and `satu_gyro` in `point_lio_mid360.yaml` to match your IMU’s specs (see Point-LIO README).

---

## 7. Saving a map

In `point_lio_mid360.yaml` you can set:

```yaml
pcd_save:
  pcd_save_en: true
  interval: -1
```

Scans will be accumulated and saved when the node is stopped (e.g. to `Point-LIO/PCD/scans.pcd` depending on the node’s cwd, or as documented in point_lio_ros2).

---

## 8. Summary (Foxy)

| Step | Where | Command / action |
|------|--------|-------------------|
| 1 | System | `sudo apt install ros-foxy-pcl-ros ros-foxy-pcl-conversions ros-foxy-visualization-msgs libeigen3-dev` |
| 2 | Any | **Livox-SDK2** – usually already installed from g1_sensor_bridge; if not, build and install |
| 3 | `~/ws_livox` | Clone **livox_ros_driver2** into `src/`, run `build.sh ROS2`, `source install/setup.bash` |
| 4 | Jetson | Copy **state_e** (see [COPY_TO_JETSON.md](COPY_TO_JETSON.md)), `git submodule update --init --recursive` |
| 5 | `g1_slam_nav` | `source ws_livox/install/setup.bash` then `colcon build --symlink-install` |
| 6 | Run | `./scripts/point_lio_start.sh` or two terminals: `point_lio_livox.sh` then `point_lio_mapping.sh` |

After this, you have everything needed to run Point-LIO with Livox Mid-360 on the Jetson.
