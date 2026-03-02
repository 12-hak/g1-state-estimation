# What to Copy to the Jetson (Point-LIO, Foxy)

Copy the following. The **state_e** tree you copy should already have the Point-Lio branch and submodule initialized.

**If you already ran the previous SLAM (KISS-ICP) on the Jetson:** You almost certainly have **Livox-SDK2** installed (used by `g1_sensor_bridge`’s livox_bridge_node). You only need to add **livox_ros_driver2** for Point-LIO (it publishes CustomMsg with per-point timestamps; the existing bridge publishes PointCloud2 only).

---

## 1. From your PC (state_e repo)

Copy the **whole state_e repo** (or at least the parts below). The Jetson should have the same branch and submodule.

**Minimum to copy:**
- `state_e/` (entire repo, including `.git`)
- Ensure branch is **Point-Lio** and submodule is initialized:
  - `g1_slam_nav/src/point_lio_ros2/` must be present (not an empty dir with only `.git`)

**Easiest:** Copy the full `state_e` folder (e.g. with rsync, scp, or USB). On the Jetson:
```bash
cd /path/where/you/copied/state_e
git status
git submodule status
git submodule update --init --recursive   # if point_lio_ros2 is not populated
```

You do **not** need to copy `g1_slam_nav/build/` or `g1_slam_nav/install/` — build those on the Jetson after copying.

---

## 2. Livox stack (build on Jetson)

**Livox-SDK2:** If you already built the previous G1 SLAM (g1_sensor_bridge), you have this — it uses `livox_lidar_sdk_shared` and `LIVOX_SDK_PATH`. Skip to livox_ros_driver2.

If not: build and install from [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) (see POINT_LIO_JETSON.md).

**livox_ros_driver2:** Needed only for Point-LIO (CustomMsg + per-point timestamps). Build on Jetson:

**Option A – Clone on Jetson:**
- Clone into `~/ws_livox/src/`, then from `~/ws_livox`: `source /opt/ros/foxy/setup.bash` and `./src/livox_ros_driver2/build.sh ROS2`.

**Option B – Copy source from PC:**
- Copy **livox_ros_driver2** into `~/ws_livox/src/livox_ros_driver2` on the Jetson, then from `~/ws_livox` run the same `build.sh ROS2`.

---

## 3. Summary: what to copy vs build on Jetson

| Item | Copy from PC? | On Jetson |
|------|----------------|-----------|
| **state_e** (with Point-Lio + submodule) | Yes – full repo | `git submodule update --init --recursive` if needed, then build `g1_slam_nav` |
| **Livox-SDK2** | Usually already there from g1_sensor_bridge | If missing: build and install |
| **livox_ros_driver2** | Optional (source only) | Build in `~/ws_livox` with `build.sh ROS2` (required for Point-LIO) |
| **g1_slam_nav/build, install** | No | Build with colcon after copying state_e |

---

## 4. After copy: build on Jetson (Foxy)

```bash
# 1) Livox-SDK2 – only if not already installed (e.g. you never built g1_sensor_bridge)
# cd Livox-SDK2 && mkdir build && cd build && cmake .. && make -j && sudo make install

# 2) livox_ros_driver2 (once – needed for Point-LIO CustomMsg)
source /opt/ros/foxy/setup.bash
cd ~/ws_livox && ./src/livox_ros_driver2/build.sh ROS2
source install/setup.bash

# 3) g1_slam_nav (once, after copying state_e)
source /opt/ros/foxy/setup.bash
source ~/ws_livox/install/setup.bash
cd /path/to/state_e/g1_slam_nav
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**Start Point-LIO (Foxy):**
```bash
cd /path/to/state_e/g1_slam_nav
chmod +x scripts/*.sh
./scripts/point_lio_start.sh
```
Or use two terminals: `./scripts/point_lio_livox.sh` then `./scripts/point_lio_mapping.sh`. See [POINT_LIO_JETSON.md](POINT_LIO_JETSON.md) for details.

**If you see `cannot execute binary file: Exec format error`:** The scripts were saved with Windows line endings (CRLF). On the Jetson, convert them and try again:
```bash
sudo apt install -y dos2unix
dos2unix /path/to/state_e/g1_slam_nav/scripts/*.sh
chmod +x /path/to/state_e/g1_slam_nav/scripts/*.sh
./scripts/point_lio_livox.sh
```
