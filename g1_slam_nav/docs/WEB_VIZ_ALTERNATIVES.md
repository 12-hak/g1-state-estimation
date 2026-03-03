# Web visualization alternatives

If the built-in web UI is blank or you want a more capable 3D view in the browser, these options work with ROS 2 and point clouds.

## Foxglove (recommended)

- **What:** Browser-based (and desktop) visualization; official ROS 2–recommended option.
- **Use:** Open [app.foxglove.dev](https://app.foxglove.dev), add the **Foxglove WebSocket** connection to `ws://<robot-ip>:8765`.
- **This repo:** Use the dedicated launch and G1 model setup: see **[FOXGLOVE_SETUP.md](FOXGLOVE_SETUP.md)**.  
  `ros2 launch g1_bringup point_lio_foxglove.launch.py` starts Point-LIO + Foxglove bridge + a minimal G1 URDF for the 3D panel.
- **Install bridge on robot:** `sudo apt install ros-humble-foxglove-bridge`
- **Pros:** 3D panel, point clouds, path, TF, robot model; free for individual/academic use.
- **Docs:** [Visualizing ROS 2 data with Foxglove](https://docs.ros.org/en/rolling/Related-Projects/Visualizing-ROS-2-Data-With-Foxglove.html)

## RVizWeb

- **What:** RViz-like interface in the browser (OSRF).
- **Repo:** [osrf/rvizweb](https://github.com/osrf/rvizweb)
- **Pros:** Familiar RViz-style UI in the browser.
- **Note:** Check repo for ROS 2 and dependency status.

## Other options

| Tool | Use case |
|------|----------|
| **PCDEditor** | Browser point cloud editor (WebGL 2); good for inspecting/editing PCDs. |
| **pointcloud-web-viewer** | Lightweight WebSocket point cloud viewer. |
| **PyViz3D** | Web-based 3D/point cloud viewer. |

## If this repo’s web UI is blank

1. **Check console:** Open DevTools (F12) → Console. A red error often means a JS/TypeScript bug or failed build; fix the reported file/line.
2. **Rebuild:** In `g1_slam_nav/src/g1_web_interface/frontend` run `npm install` then `npm run build`. Serve the built files (e.g. from the backend’s static folder).
3. **Data:** The canvas shows grid + robot + points when the bridge sends `pose` and either `map_cloud` or `status` (scan). If no data is sent yet, you’ll see the grid and robot only after the first pose.

For full 3D and minimal custom code, **Foxglove + foxglove-bridge** is the most straightforward alternative to RViz in the browser.
