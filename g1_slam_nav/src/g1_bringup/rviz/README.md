# RViz configs

## point_lio.rviz

Used for Point-LIO mapping (Livox + `Laser_map`, `cloud_registered`, path).

### Rotating the view (orbit)

- **Shift + left mouse drag** — orbit/rotate the camera around the focal point.
- **Left mouse drag (no Shift)** — move the view (pan).
- **Scroll** — zoom in/out.

If rotation doesn’t seem to work, make sure you’re holding **Shift** while dragging with the left button. The view is set to **Orbit**, so the focal point is the origin of the Fixed Frame (`camera_init`). To orbit around the robot, in the **Views** panel you can right‑click the current view and use “Move camera to look at” after selecting a frame, or focus the view on the map first.

### Dense/blocky scan where the robot is

**CloudRegistered** uses **Decay Time: 30** so you still see where the robot has been. **Size (m)** is set to **0.005 m** so overlapping points read as a smoother density. If it’s still too chunky when standing still, lower **Size (m)** further (e.g. 0.004 or 0.003) in the PointCloud2 display properties.

The **web UI** and **Foxglove** match this setup; the web UI also voxel-downsamples the 30 s scan layer (one point per 0.02 m voxel) to reduce blockiness when the robot is stationary. See **docs/FOXGLOVE_SETUP.md** for parity across RViz, web, and Foxglove.

If the **map** (CloudMap) looks too dense, increase **filter_size_map** in `point_lio_mapping.launch.py` (e.g. to `0.3` or `0.4`).
