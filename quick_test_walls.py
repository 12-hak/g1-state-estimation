#!/usr/bin/env python3
"""Quick test of wall detection algorithms."""

import numpy as np
import time
from point_cloud_to_walls import quick_wall_detection

# Generate simple test data - a rectangular room
print("Generating test room...")
points = []

# Four walls of a 5x5m room
for i in range(200):
    # North wall
    x = np.random.uniform(-2.5, 2.5)
    z = np.random.uniform(0, 1.5)
    points.append([x, 2.5 + np.random.normal(0, 0.02), z])
    
    # South wall
    points.append([x, -2.5 + np.random.normal(0, 0.02), z])
    
    # East wall
    y = np.random.uniform(-2.5, 2.5)
    points.append([2.5 + np.random.normal(0, 0.02), y, z])
    
    # West wall
    points.append([-2.5 + np.random.normal(0, 0.02), y, z])

points = np.array(points)
print(f"Generated {len(points)} points")

# Detect walls
print("\nDetecting walls...")
start = time.time()
segments = quick_wall_detection(points)
elapsed = (time.time() - start) * 1000

print(f"Detection time: {elapsed:.2f}ms")
print(f"Detected {len(segments)} wall segments:")

for i, seg in enumerate(segments):
    length = np.linalg.norm(seg.end - seg.start)
    print(f"\n  Wall {i+1}:")
    print(f"    Length: {length:.2f}m")
    print(f"    Thickness: {seg.thickness:.3f}m")
    print(f"    Points: {len(seg.points)}")

print("\n✓ Test completed successfully!")
