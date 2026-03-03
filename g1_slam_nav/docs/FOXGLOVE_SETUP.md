# Foxglove setup for Point-LIO (point cloud, path, G1 model)

View Point-LIO data in **Foxglove** (browser or desktop) with the same effective quality as RViz: point clouds (map + scan, AxisColor), trajectory path, and a Unitree G1 robot model. The **web UI** (same launch, port 8080) also matches RViz and adds voxel-downsampling on the scan layer to reduce blockiness when the robot stands still.

## 1. Install Foxglove bridge

On the robot (or machine running ROS 2):

```bash
sudo apt install ros-humble-foxglove-bridge
```

(Use your ROS distro: `ros-<distro>-foxglove-bridge`.)

## 2. Launch Point-LIO with Foxglove

```bash
# Terminal 1: Livox driver (if not already running)
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# Terminal 2: Point-LIO + Foxglove bridge + optional web UI
ros2 launch g1_bringup point_lio_foxglove.launch.py
```

Options:

- `foxglove:=false` — do not start the Foxglove bridge (e.g. if you only want the web UI).
- `use_web:=false` — do not start the web UI.

The Foxglove WebSocket server listens on **port 8765** (all interfaces).

## 3. Connect from Foxglove

1. Open **[app.foxglove.dev](https://app.foxglove.dev)** (or Foxglove Studio desktop).
2. **Open connection** → **Foxglove WebSocket**.
3. URL: `ws://<robot-ip>:8765` (replace `<robot-ip>` with the machine running the launch, e.g. `192.168.123.164`).
4. Click **Open**.

## 4. Add panels (point cloud, path, G1 model) — match RViz quality

- **3D** panel:
  - **Fixed frame**: `camera_init` (Point-LIO world frame).
  - **PointCloud2** (to match RViz `point_lio.rviz`):
    - **Map**: Add topic `/Laser_map`. Color: **AxisColor** (height). Point size: **0.015** m (same as RViz CloudMap). No decay.
    - **Scan**: Add topic `/cloud_registered`. Color: **AxisColor**. Point size: **0.005** m (or **0.004** if the scan looks blocky when the robot stands still). In Foxglove you can set a decay time (e.g. 30 s) if the panel supports it, so only recent scans are shown.
  - **Path**: Add topic `/path` (trajectory so far).
  - **URDF / Robot model**:
    - Add a **URDF** (or **Model**) layer; source **Topic**, topic `/robot_description`.
    - The minimal G1 is rooted at `aft_mapped`, so it follows the robot pose.

**Reduce blockiness when standing:** If the scan near the robot looks thick or blocky, lower the **point size** for `/cloud_registered` (e.g. to 0.004 or 0.003 m) in the 3D panel settings. That matches what we do in the web UI (smaller scan points + voxel-downsample).

## 5. G1 model (minimal vs full)

- **Minimal (default):** The launch publishes a minimal URDF (`g1_minimal.urdf`) to `/robot_description`: a single link `aft_mapped` with a box geometry approximating the G1 torso. No meshes required.

- **Full G1 from Unitree:** For the full Unitree G1 mesh model:
  1. Clone [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) and build the `g1_description` package.
  2. Run `robot_state_publisher` with the G1 URDF (e.g. `g1_29dof.urdf`) and ensure TF `camera_init` → `aft_mapped` (or your base link) is published by Point-LIO.
  3. Either publish that URDF to `/robot_description` (e.g. with a similar script pointing at the unitree URDF) or, in Foxglove desktop, use **URDF from URL** or **file** if you have the URDF and meshes available.

## 6. Parity with RViz and web UI

| Item | RViz (point_lio.rviz) | Web UI | Foxglove |
|------|------------------------|--------|----------|
| Map cloud | /Laser_map, AxisColor, 0.015 m | Same | Set point size 0.015 m, AxisColor |
| Scan cloud | /cloud_registered, AxisColor, 0.005 m, 30 s decay | 0.004 m, 30 s decay, voxel 0.02 m | 0.005 m (or 0.004), AxisColor; use decay if available |
| Path | /path | /path (trajectory) | /path |
| Blockiness when standing | Small point size (0.005 m) | Voxel-downsample + 0.004 m | Lower point size for scan in panel settings |

All three can deliver the same effective quality; the web UI adds voxel-downsampling on the scan layer to keep the “where I’ve been” trail while reducing local blockiness.

## 7. Troubleshooting

- **No point cloud / path:** Check that Point-LIO is running and publishing `/Laser_map`, `/cloud_registered`, and `/path`. Use `ros2 topic list` and `ros2 topic hz /Laser_map`.
- **Robot model not visible:** Ensure fixed frame is `camera_init` and TF `camera_init` → `aft_mapped` is being published (Point-LIO does this). Check **TF** in the 3D panel.
- **Connection refused:** Ensure the Foxglove bridge is started (`foxglove:=true`) and no firewall is blocking port 8765.
