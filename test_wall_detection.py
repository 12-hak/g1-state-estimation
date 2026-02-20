#!/usr/bin/env python3
"""
Test script for point cloud to wall conversion algorithms.

Demonstrates the various algorithms with synthetic and real data.
"""

import numpy as np
import time
from point_cloud_to_walls import (
    PointCloudToWalls,
    AdaptiveWallRenderer,
    quick_wall_detection,
    quick_plane_detection
)


def generate_test_room(room_size=5.0, wall_thickness=0.1, points_per_meter=50):
    """
    Generate a synthetic rectangular room point cloud.
    
    Args:
        room_size: Room dimensions (square room)
        wall_thickness: Wall thickness in meters
        points_per_meter: Point density
        
    Returns:
        Nx3 numpy array of points
    """
    points = []
    
    # Four walls
    walls = [
        # North wall (y = room_size/2)
        lambda x, z: np.array([x, room_size/2, z]),
        # South wall (y = -room_size/2)
        lambda x, z: np.array([x, -room_size/2, z]),
        # East wall (x = room_size/2)
        lambda x, z: np.array([room_size/2, x, z]),
        # West wall (x = -room_size/2)
        lambda x, z: np.array([-room_size/2, x, z]),
    ]
    
    # Generate points on each wall
    num_points = int(room_size * points_per_meter)
    for wall_func in walls:
        for _ in range(num_points):
            x = np.random.uniform(-room_size/2, room_size/2)
            z = np.random.uniform(0, 1.5)  # 1.5m tall walls
            # Add some noise
            point = wall_func(x, z)
            point += np.random.normal(0, wall_thickness/2, 3)
            points.append(point)
    
    return np.array(points)


def generate_test_corridor(length=10.0, width=2.0, points_per_meter=50):
    """Generate a corridor with two parallel walls."""
    points = []
    num_points = int(length * points_per_meter)
    
    # Left wall
    for _ in range(num_points):
        x = np.random.uniform(0, length)
        z = np.random.uniform(0, 1.5)
        y = -width/2 + np.random.normal(0, 0.02)
        points.append([x, y, z])
    
    # Right wall
    for _ in range(num_points):
        x = np.random.uniform(0, length)
        z = np.random.uniform(0, 1.5)
        y = width/2 + np.random.normal(0, 0.02)
        points.append([x, y, z])
    
    return np.array(points)


def test_wall_detection():
    """Test wall detection on synthetic room."""
    print("=" * 60)
    print("TEST 1: Wall Detection on Synthetic Room")
    print("=" * 60)
    
    # Generate test data
    print("\nGenerating synthetic room (5x5m)...")
    points = generate_test_room(room_size=5.0, points_per_meter=100)
    print(f"Generated {len(points)} points")
    
    # Detect walls
    print("\nDetecting walls...")
    start_time = time.time()
    segments = quick_wall_detection(points)
    elapsed = (time.time() - start_time) * 1000
    
    print(f"Detection time: {elapsed:.2f}ms")
    print(f"Detected {len(segments)} wall segments:")
    
    for i, seg in enumerate(segments):
        length = np.linalg.norm(seg.end - seg.start)
        print(f"\n  Wall {i+1}:")
        print(f"    Length: {length:.2f}m")
        print(f"    Thickness: {seg.thickness:.3f}m")
        print(f"    Points: {len(seg.points)}")
        print(f"    Start: [{seg.start[0]:.2f}, {seg.start[1]:.2f}, {seg.start[2]:.2f}]")
        print(f"    End: [{seg.end[0]:.2f}, {seg.end[1]:.2f}, {seg.end[2]:.2f}]")
        print(f"    Normal: [{seg.normal[0]:.2f}, {seg.normal[1]:.2f}, {seg.normal[2]:.2f}]")


def test_plane_detection():
    """Test plane detection on synthetic room."""
    print("\n" + "=" * 60)
    print("TEST 2: Plane Detection on Synthetic Room")
    print("=" * 60)
    
    # Generate test data
    print("\nGenerating synthetic room (5x5m)...")
    points = generate_test_room(room_size=5.0, points_per_meter=100)
    print(f"Generated {len(points)} points")
    
    # Detect planes
    print("\nDetecting planes...")
    start_time = time.time()
    planes = quick_plane_detection(points)
    elapsed = (time.time() - start_time) * 1000
    
    print(f"Detection time: {elapsed:.2f}ms")
    print(f"Detected {len(planes)} planes:")
    
    for i, plane in enumerate(planes):
        print(f"\n  Plane {i+1}:")
        print(f"    Normal: [{plane.normal[0]:.2f}, {plane.normal[1]:.2f}, {plane.normal[2]:.2f}]")
        print(f"    Distance: {plane.distance:.2f}m")
        print(f"    Confidence: {plane.confidence:.2%}")
        print(f"    Points: {len(plane.points)}")
        print(f"    Bounds: X[{plane.bounds[0]:.2f}, {plane.bounds[1]:.2f}] "
              f"Y[{plane.bounds[2]:.2f}, {plane.bounds[3]:.2f}] "
              f"Z[{plane.bounds[4]:.2f}, {plane.bounds[5]:.2f}]")


def test_corridor():
    """Test on corridor scenario."""
    print("\n" + "=" * 60)
    print("TEST 3: Corridor Detection")
    print("=" * 60)
    
    # Generate corridor
    print("\nGenerating corridor (10m x 2m)...")
    points = generate_test_corridor(length=10.0, width=2.0, points_per_meter=50)
    print(f"Generated {len(points)} points")
    
    # Detect walls
    print("\nDetecting walls...")
    start_time = time.time()
    segments = quick_wall_detection(points)
    elapsed = (time.time() - start_time) * 1000
    
    print(f"Detection time: {elapsed:.2f}ms")
    print(f"Detected {len(segments)} wall segments (expected 2):")
    
    for i, seg in enumerate(segments):
        length = np.linalg.norm(seg.end - seg.start)
        print(f"\n  Wall {i+1}:")
        print(f"    Length: {length:.2f}m")
        print(f"    Thickness: {seg.thickness:.3f}m")
        print(f"    Points: {len(seg.points)}")


def test_performance():
    """Test performance on various point cloud sizes."""
    print("\n" + "=" * 60)
    print("TEST 4: Performance Benchmarks")
    print("=" * 60)
    
    sizes = [100, 500, 1000, 2000, 5000]
    
    print("\nPoint Count | Detection Time | Walls Found")
    print("-" * 50)
    
    for size in sizes:
        # Generate random points in a room
        points = generate_test_room(room_size=5.0, points_per_meter=size//20)
        
        # Time detection
        start_time = time.time()
        segments = quick_wall_detection(points)
        elapsed = (time.time() - start_time) * 1000
        
        print(f"{len(points):>11} | {elapsed:>13.2f}ms | {len(segments):>11}")


def test_adaptive_renderer():
    """Test adaptive wall renderer."""
    print("\n" + "=" * 60)
    print("TEST 5: Adaptive Wall Renderer")
    print("=" * 60)
    
    # Create renderer
    renderer = AdaptiveWallRenderer()
    
    # Generate test data
    print("\nGenerating test room...")
    points = generate_test_room(room_size=5.0, points_per_meter=100)
    
    # Simulate multiple updates
    print("\nSimulating real-time updates:")
    for i in range(5):
        current_time = time.time()
        
        # Add some noise to simulate movement
        noisy_points = points + np.random.normal(0, 0.01, points.shape)
        
        start = time.time()
        segments = renderer.update_walls(noisy_points, current_time)
        elapsed = (time.time() - start) * 1000
        
        print(f"  Update {i+1}: {elapsed:.2f}ms, {len(segments)} walls")
        
        # Get render primitives
        primitives = renderer.get_render_primitives(segments, wall_height=1.5)
        print(f"    Generated {len(primitives)} render primitives")
        
        time.sleep(0.1)  # Simulate frame delay


def test_custom_config():
    """Test with custom configuration."""
    print("\n" + "=" * 60)
    print("TEST 6: Custom Configuration")
    print("=" * 60)
    
    # Generate test data
    points = generate_test_room(room_size=5.0, points_per_meter=100)
    
    # Test different configurations
    configs = [
        ("Default", {}),
        ("High Precision", {
            "voxel_size": 0.02,
            "ransac_threshold": 0.02,
            "dbscan_eps": 0.1
        }),
        ("Fast", {
            "voxel_size": 0.1,
            "min_points_per_wall": 20,
            "dbscan_eps": 0.3
        }),
    ]
    
    print("\nConfiguration | Time (ms) | Walls | Avg Points/Wall")
    print("-" * 65)
    
    for name, config in configs:
        detector = PointCloudToWalls(**config)
        
        start_time = time.time()
        segments = detector.detect_walls_fast(points)
        elapsed = (time.time() - start_time) * 1000
        
        avg_points = np.mean([len(s.points) for s in segments]) if segments else 0
        
        print(f"{name:>13} | {elapsed:>9.2f} | {len(segments):>5} | {avg_points:>15.1f}")


def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("POINT CLOUD TO WALL CONVERSION - TEST SUITE")
    print("=" * 60)
    
    try:
        test_wall_detection()
        test_plane_detection()
        test_corridor()
        test_performance()
        test_adaptive_renderer()
        test_custom_config()
        
        print("\n" + "=" * 60)
        print("ALL TESTS COMPLETED SUCCESSFULLY")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
