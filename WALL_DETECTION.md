# Point Cloud to Wall Conversion - High-Speed Algorithms

This module provides high-performance algorithms for converting LiDAR point clouds into geometric wall representations for real-time robot visualization.

## Features

### 🚀 High-Speed Algorithms

1. **RANSAC Plane Detection**
   - Robust plane fitting using Random Sample Consensus
   - Iteratively detects multiple planes in the scene
   - Configurable inlier threshold and iteration count

2. **DBSCAN Clustering**
   - Fast spatial clustering to group wall segments
   - Automatically separates distinct structures
   - Noise filtering built-in

3. **Voxel Grid Downsampling**
   - Efficient point cloud reduction
   - Preserves spatial structure
   - Configurable voxel size

4. **Line Segment Extraction**
   - PCA-based line fitting for 2D wall segments
   - Automatic thickness estimation
   - Normal vector computation

### 🎨 Rendering Styles

The visualization focuses on clarity and performance:

- **Vertical Posts**: Walls and obstacles are rendered as thin vertical cylinders or boxes (default: 3cm radius). This prevents the "big block" confusion of solid wall geometry.
- **Color Gradient**: Markers are colored by distance (White/Yellow for close, Orange for far) to help with depth perception.
- **Adaptive Downsampling**: Faraway points are rendered with lower density to maintain a high framerate.
- **Breadcrumbs**: A sphere-based path trace can be enabled independently of map markers.

## Usage

### Command Line

Run the visualizer with your preferred options:

```bash
# Standard viewing (Walls enabled, max 1500 points)
python g1_network_viz.py

# Cleanest view (Breadcrumbs only, walls disabled)
python g1_network_viz.py --no-walls --breadcrumbs

# High-performance mode (Useful for remote desktop or slow PCs)
python g1_network_viz.py --max-points 500
```

### Python API

```python
from point_cloud_to_walls import (
    PointCloudToWalls, 
    AdaptiveWallRenderer,
    quick_wall_detection,
    quick_plane_detection
)

# Quick detection with defaults
import numpy as np
points = np.random.rand(1000, 3)  # Your point cloud

# Detect wall segments
segments = quick_wall_detection(points)

# Detect planes
planes = quick_plane_detection(points)

# Custom configuration
detector = PointCloudToWalls(
    voxel_size=0.05,           # 5cm voxels
    min_points_per_wall=10,    # Minimum points to consider
    ransac_threshold=0.05,     # 5cm inlier threshold
    dbscan_eps=0.15,           # 15cm clustering radius
    dbscan_min_samples=5       # Minimum cluster size
)

walls = detector.detect_walls_fast(points)
```

### Advanced Usage

```python
# Create adaptive renderer for real-time updates
renderer = AdaptiveWallRenderer()

# In your render loop
current_time = time.time()
wall_segments = renderer.update_walls(point_cloud, current_time)

# Get rendering primitives
primitives = renderer.get_render_primitives(wall_segments, wall_height=1.5)

# Each primitive contains:
# - type: 'box'
# - position: [x, y, z]
# - size: [length/2, thickness/2, height/2]
# - rotation: 3x3 rotation matrix
# - color: [r, g, b, a]
# - alpha: transparency
```

## Algorithm Details

### Wall Detection Pipeline

1. **Voxel Downsampling** (O(n))
   - Reduces point count while preserving structure
   - Typical reduction: 50-90% depending on density

2. **DBSCAN Clustering** (O(n log n))
   - Groups points into separate wall structures
   - Filters noise automatically
   - Uses only X,Y coordinates (assumes vertical walls)

3. **Line Segment Fitting** (O(n) per cluster)
   - PCA-based principal direction extraction
   - Projects points onto fitted line
   - Estimates wall thickness from perpendicular distances

### Performance Characteristics

- **Voxel Downsampling**: ~0.5ms for 1000 points
- **DBSCAN Clustering**: ~2-5ms for 1000 points
- **Line Fitting**: ~0.1ms per cluster
- **Total Pipeline**: ~5-10ms for typical scans (real-time capable)

### RANSAC Plane Detection

For more complex scenes, use plane detection:

```python
planes = detector.detect_planes_ransac(points, max_planes=5)

for plane in planes:
    print(f"Normal: {plane.normal}")
    print(f"Distance: {plane.distance}")
    print(f"Confidence: {plane.confidence}")
    print(f"Bounds: {plane.bounds}")
    print(f"Points: {len(plane.points)}")
```

## Configuration Parameters

### PointCloudToWalls

| Parameter | Default | Description |
|-----------|---------|-------------|
| `voxel_size` | 0.05 | Voxel grid size (meters) |
| `min_points_per_wall` | 10 | Minimum points to detect a wall |
| `ransac_threshold` | 0.05 | RANSAC inlier distance (meters) |
| `dbscan_eps` | 0.15 | DBSCAN clustering radius (meters) |
| `dbscan_min_samples` | 5 | DBSCAN minimum cluster size |

### AdaptiveWallRenderer

| Parameter | Default | Description |
|-----------|---------|-------------|
| `update_interval` | 0.5 | Geometry update frequency (seconds) |

Adjust these based on your needs:
- **Higher accuracy**: Decrease `voxel_size`, `ransac_threshold`, `dbscan_eps`
- **Better performance**: Increase `voxel_size`, decrease `min_points_per_wall`
- **More segments**: Decrease `dbscan_eps`, `dbscan_min_samples`

## Visualization Features

### Distance-Based Coloring

Walls are colored based on distance from robot:
- **Red** (0-1m): Close obstacles
- **Yellow** (1-2m): Medium distance
- **Green** (2m+): Far walls

### Adaptive Detail

- Close walls (< 1m): Full detail, all points
- Medium walls (1-2m): 50% downsampling
- Far walls (> 2m): 10% downsampling

### Wall Properties

Each detected wall segment includes:
- **Start/End points**: 3D coordinates
- **Normal vector**: Perpendicular direction
- **Thickness**: Estimated from point spread
- **Confidence**: Based on point count and fit quality

## Integration with Existing Code

The wall detection is seamlessly integrated into `g1_network_viz.py`:

1. **Automatic Updates**: Walls are recomputed every 0.5s
2. **Persistent Tracking**: Existing wall persistence logic is preserved
3. **Mode Switching**: Easy toggle between visualization styles
4. **Performance**: Minimal overhead, runs at 50 Hz

## Troubleshooting

### No walls detected

- Check `min_points_per_wall` - may be too high
- Verify point cloud has sufficient density
- Try decreasing `dbscan_eps` for tighter clustering

### Too many small segments

- Increase `dbscan_eps` to merge nearby clusters
- Increase `min_points_per_wall` to filter small detections
- Increase `voxel_size` for more aggressive downsampling

### Performance issues

- Increase `update_interval` to reduce computation frequency
- Increase `voxel_size` for faster downsampling
- Use 'walls' mode instead of 'hybrid' to skip point rendering

## Future Enhancements

Potential improvements:
- [ ] Alpha shapes for polygon extraction
- [ ] Mesh simplification for complex walls
- [ ] Corner detection and room segmentation
- [ ] Wall texture/material classification
- [ ] Multi-threaded processing for large scans
- [ ] GPU acceleration for clustering

## Dependencies

- `numpy` - Core array operations and math
- `mujoco` - Physics engine and real-time visualizer

## References

- RANSAC: Fischler & Bolles (1981)
- DBSCAN: Ester et al. (1996)
- PCA Line Fitting: Pearson (1901)
