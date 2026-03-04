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

---

## 5. After a reboot: no lidar data

The Livox driver does **not** start automatically. You must source and launch it every time.

**1. Source in every terminal that uses ROS 2:**
```bash
source /opt/ros/foxy/setup.bash
source ~/ws_livox/install/setup.bash
# If you run Point-LIO or echo topics from this shell, also:
source /path/to/state_e/g1_slam_nav/install/setup.bash
```

**2. Start the Livox driver** (in terminal 1):
```bash
cd /path/to/state_e/g1_slam_nav
./scripts/point_lio_livox.sh
```
Or by hand: `ros2 launch livox_ros_driver2 msg_MID360_launch.py` (after sourcing both ROS and `~/ws_livox/install/setup.bash`).

**3. Check the topic:**
```bash
source /opt/ros/foxy/setup.bash
source ~/ws_livox/install/setup.bash
ros2 topic list
ros2 topic hz /livox/lidar
```
You should see `/livox/lidar` and a nonzero Hz. If you only see the Python deprecation warning and no messages, the driver is not running or this terminal has not sourced the Livox workspace (so the CustomMsg type may be missing).

**4. Then start Point-LIO** (terminal 2, with both workspaces sourced):
```bash
source /opt/ros/foxy/setup.bash
source ~/ws_livox/install/setup.bash
source /path/to/state_e/g1_slam_nav/install/setup.bash
./scripts/point_lio_mapping.sh
```

**If Livox is in a different path:** set `WS_LIVOX_INSTALL` before sourcing, e.g. `export WS_LIVOX_INSTALL=/path/to/ws_livox/install` and then `source $WS_LIVOX_INSTALL/setup.bash`.

---

## 6. Web interface is blank (RViz works, browser at :8080 is empty)

The bridge serves the **built** React app. If you never built it (or didn’t reinstall after building), the page can be blank or show errors.

**1. Build the frontend** (on the machine where you build the repo, e.g. Jetson or PC):

```bash
cd /path/to/state_e/g1_slam_nav/src/g1_web_interface/frontend
npm install
npm run build
```

**2. Reinstall the package** so the `dist/` output is copied into the install space:

```bash
cd /path/to/state_e/g1_slam_nav
source /opt/ros/foxy/setup.bash
source ~/ws_livox/install/setup.bash
colcon build --packages-select g1_web_interface
source install/setup.bash
```

**3. Restart the Point-LIO launch** (so the web bridge picks up the install):

```bash
./scripts/point_lio_mapping.sh
```

**4. Open the UI** at `http://<robot-ip>:8080`. You should see a black view with a green “Connected” dot once data is flowing.

**If it’s still blank:** In the terminal where the launch is running, check for either “Serving frontend from …” (good) or “Could not find frontend/index.html” (bad). In the browser, press **F12** → **Console** and look for red errors (e.g. 404 on JS files = frontend not installed correctly; WebSocket errors = wrong host/port or bridge not running).

---

## 7. Web frontend: slow copy/build and "stuck" on Jetson

**Why it's slow:** The frontend uses TypeScript + Vite + React + Three.js. `npm run build` runs `tsc` then `vite build`; on a Jetson this can take several minutes and use a lot of RAM. Copying the repo **with** `frontend/node_modules` is also slow (hundreds of MB). If the Jetson is under memory pressure, the build can appear to hang.

**Recommended: build on PC, copy only what's needed**

1. **On your PC** (Windows/Linux, in the repo):
   ```bash
   cd g1_slam_nav/src/g1_web_interface/frontend
   npm install
   npm run build
   ```
   You should see `frontend/dist/` with `index.html` and JS/CSS assets.

2. **Copy to Jetson** in either way:
   - **Option A:** Copy the whole `g1_slam_nav` tree **but exclude** `frontend/node_modules` (e.g. `rsync --exclude='node_modules'` or delete `node_modules` before copying). That way the Jetson has `frontend/dist` from the PC and never runs `npm run build`.
   - **Option B:** Copy the full tree including `node_modules`; then on Jetson only run `colcon build --packages-select g1_web_interface` (no `npm install` / `npm run build`). Use the existing `frontend/dist` from the PC.

3. **On the Jetson** (no npm needed if you used Option A or B):
   ```bash
   cd /path/to/state_e/g1_slam_nav
   source /opt/ros/foxy/setup.bash
   source ~/ws_livox/install/setup.bash
   colcon build --packages-select g1_web_interface
   source install/setup.bash
   ```

**If you already copied and the build is stuck:** A reboot can help (frees memory). Then either:
- Try `npm run build` again in `frontend/` (give it 5–10 minutes), or
- Build on the PC as above, copy only `frontend/dist` into the same path on the Jetson, then run `colcon build --packages-select g1_web_interface`.

**Summary:** For the web UI, the Jetson only needs the **built** `frontend/dist/` (from PC or from a successful npm build on Jetson). It does not need `node_modules` to run the app; colcon just installs the contents of `dist/` into the ROS install space.

---

## 8. Two-client setup: Point-LIO on Jetson, Nav2 + web on another machine

To keep localization (Point-LIO) fast and avoid Nav2/planning load on the robot, you can run **Point-LIO + occupancy grid** on the Jetson and **Nav2 + web bridge** on a second machine (PC or another Jetson) on the same network.

**On the Jetson (robot):**

1. Source ROS + Livox + this workspace.
2. Start Livox driver: `./scripts/point_lio_livox.sh` (or `ros2 launch livox_ros_driver2 msg_MID360_launch.py`).
3. Start Point-LIO with occupancy grid and map manager (for save_map service):
   ```bash
   ros2 launch g1_bringup point_lio_mapping.launch.py use_occ_grid:=true use_web:=false
   ```
   This runs: Point-LIO, `point_lio_occupancy_grid_node` (Laser_map → /map), `map_manager_node`, and **point_lio_odom_to_tf** (subscribes to `aft_mapped_to_init`, publishes TF `camera_init` → `base_link`) so Nav2 on the client sees a single TF tree. Set `use_web:=true` if you also want the web UI on the Jetson.

4. Ensure the second machine can reach the Jetson’s ROS 2 topics (same network, same `ROS_DOMAIN_ID` if set).

**On the second machine (PC or other Jetson):**

1. Install and build this workspace (including `g1_slam` for the occupancy grid node if you ever run it there; for the client you need at least `g1_navigation`, `g1_web_interface`, and Nav2).
2. **Python version:** ROS 2 Humble uses Python 3.10. The workspace must be built with the same Python so that `g1_msgs` type support loads. If you use conda/pyenv or have Python 3.12 as default, deactivate it before building and running (`conda deactivate`, `unset LD_LIBRARY_PATH`). Check with `python3 --version` (should be 3.10) and `which python3` (e.g. `/usr/bin/python3`). If you previously built with a different Python, clean and rebuild: `rm -rf build/g1_msgs install/g1_msgs build/g1_web_interface install/g1_web_interface` then `colcon build --symlink-install ...`.
3. Set `ROS_DOMAIN_ID` to match the Jetson (if used).
4. Launch Nav2 + web for waypoint navigation:
   ```bash
   source /opt/ros/humble/setup.bash   # client PC is typically Ubuntu 22.04 + Humble
   source /path/to/g1_slam_nav/install/setup.bash
   ros2 launch g1_bringup point_lio_navigation.launch.py
   ```
   This runs Nav2 (with `nav2_params_point_lio.yaml`, frame `camera_init`), `nav_command_node` (subscribes to `nav/goal` and `nav/waypoints` from the web), and the web bridge. The client subscribes to `/map` and tf from the robot.

**Map save:** With `map_manager_node` running on the Jetson (as above), the web “Save map” button (or a `save_map` service call) saves the current Point-LIO point cloud (Laser_map) and, if available, the occupancy grid to `/home/unitree/maps/`. Use the saved grid YAML with Nav2’s map_server when running from a saved map.

**Single machine:** You can also run everything on the Jetson: `point_lio_mapping.launch.py use_occ_grid:=true` and then `point_lio_navigation.launch.py` in a second terminal. Nav2 will use more CPU; tune planner frequency if needed.

---

**If you see `cannot execute binary file: Exec format error`:** The scripts were saved with Windows line endings (CRLF). On the Jetson, convert them and try again:
```bash
sudo apt install -y dos2unix
dos2unix /path/to/state_e/g1_slam_nav/scripts/*.sh
chmod +x /path/to/state_e/g1_slam_nav/scripts/*.sh
./scripts/point_lio_livox.sh
```
