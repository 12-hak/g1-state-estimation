# full-scan: Next Steps (Linux)

## 1. Clone / checkout

```bash
cd /path/to/your/repo
git fetch
git checkout full-scan
git pull
```

If Point-LIO submodule is empty, populate it:

```bash
cd g1_slam_nav/src/point_lio_ros2
git fetch origin
git checkout main
git checkout main -- .
cd -
```

## 2. Build (PC Linux)

Requires ROS2 (Foxy or Humble) and livox_ros_driver2 in a separate workspace for message types.

```bash
source /opt/ros/foxy/setup.bash
source ~/ws_livox/install/setup.bash   # or your livox_ros_driver2 install path
cd g1_slam_nav
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 3. Jetson: sensors only

On the G1 Jetson, start only the Livox driver (so PC can subscribe to raw topics):

```bash
export ROS_DOMAIN_ID=10
./scripts/point_lio_livox.sh
```

Do **not** run Point-LIO mapping on Jetson.

## 4. PC: run mapper

From repo root, same ROS_DOMAIN_ID:

```bash
export ROS_DOMAIN_ID=10
export G1_WS=/path/to/repo/g1_slam_nav
./full-scan/run_pc_pointlio.sh
# Optional: ./full-scan/run_pc_pointlio.sh rviz:=true
```

## 5. Verify

In another terminal (after sourcing install):

```bash
export ROS_DOMAIN_ID=10
./full-scan/check_pc_pointlio.sh
```

Expect: `/livox/lidar`, `/livox/imu` (from Jetson), `/aft_mapped_to_init`, `/Laser_map`, `/cloud_registered` (from PC mapper).

## 6. Web UI (optional)

If you launched with `use_web:=true` (default), open:

`http://<pc-ip>:8080`

WS port 9090. Point-LIO mode is enabled in the bridge.

## Troubleshooting

- **No lidar/imu on PC:** Same LAN, same ROS_DOMAIN_ID, firewall allows DDS.
- **point_lio not found:** Ensure `point_lio_ros2` has content and is built; source livox workspace before colcon build.
- **Port in use:** `./full-scan/run_pc_pointlio.sh ws_port:=9091 http_port:=8081`
