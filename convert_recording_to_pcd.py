#!/usr/bin/env python3
"""
Convert G1 LiDAR recordings to PLY/PCD format for viewing in CloudCompare/MeshLab.

Usage:
    python convert_recording_to_pcd.py recordings/recording_20260111_110530 [--format ply]
    
This will read:
    - recording_20260111_110530_poses.txt
    - recording_20260111_110530_clouds.bin
    
And output:
    - recording_20260111_110530.ply (or .pcd)
"""

import struct
import numpy as np
import sys
from pathlib import Path

def read_poses(pose_file):
    """Read pose file (timestamp x y z qw qx qy qz)"""
    poses = {}
    with open(pose_file, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) == 8:
                timestamp = int(parts[0])
                position = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                quaternion = np.array([float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])])
                poses[timestamp] = (position, quaternion)
    return poses

def quaternion_to_rotation_matrix(q):
    """Convert quaternion (w, x, y, z) to 3x3 rotation matrix"""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])

def read_clouds(cloud_file, poses):
    """Read binary cloud file and transform to world frame"""
    all_points = []
    matched_frames = 0
    unmatched_frames = 0
    
    with open(cloud_file, 'rb') as f:
        frame_count = 0
        while True:
            # Read frame header: timestamp (8 bytes) + num_points (4 bytes)
            header = f.read(12)
            if len(header) < 12:
                break
            
            timestamp, num_points = struct.unpack('<QI', header)
            
            # Find closest pose
            if timestamp not in poses:
                # Try to find closest timestamp within 100ms
                closest_ts = min(poses.keys(), key=lambda x: abs(x - timestamp), default=None)
                if closest_ts and abs(closest_ts - timestamp) < 100000:  # 100ms tolerance
                    timestamp = closest_ts
                    if frame_count == 0:
                        print(f"  Using closest pose match (delta: {abs(closest_ts - timestamp)/1000:.1f}ms)")
                else:
                    # Skip this frame if no pose
                    f.read(num_points * 12)  # 3 floats per point
                    unmatched_frames += 1
                    continue
            
            position, quaternion = poses[timestamp]
            
            # Debug first frame
            if frame_count == 0:
                print(f"  First frame pose: pos={position}, quat={quaternion}")
            
            # Debug every 20 frames to see if pose is changing
            if frame_count > 0 and frame_count % 20 == 0:
                print(f"  Frame {frame_count} pose: pos={position[:2]} (x,y only)")
            
            R = quaternion_to_rotation_matrix(quaternion)
            
            # Read points (x, y, z floats)
            points_data = f.read(num_points * 12)
            if len(points_data) < num_points * 12:
                break
            
            points = np.frombuffer(points_data, dtype=np.float32).reshape(-1, 3)
            
            # Transform to world frame
            points_world = (R @ points.T).T + position
            all_points.append(points_world)
            
            matched_frames += 1
            frame_count += 1
            if frame_count % 10 == 0:
                print(f"  Processed {frame_count} frames, {len(all_points[-1])} points in last frame...")
    
    print(f"  Matched: {matched_frames} frames, Unmatched: {unmatched_frames} frames")
    
    if not all_points:
        return np.array([])
    
    return np.vstack(all_points)

def write_ply(points, output_file):
    """Write points to PLY file (binary format for smaller size)"""
    with open(output_file, 'w') as f:
        # PLY Header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        
        # Points
        for pt in points:
            f.write(f"{pt[0]} {pt[1]} {pt[2]}\n")
    
    print(f"Wrote {len(points)} points to {output_file}")

def write_pcd(points, output_file):
    """Write points to PCD file"""
    with open(output_file, 'w') as f:
        # PCD Header
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        
        # Points
        for pt in points:
            f.write(f"{pt[0]} {pt[1]} {pt[2]}\n")
    
    print(f"Wrote {len(points)} points to {output_file}")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Convert G1 LiDAR recordings to point cloud format')
    parser.add_argument('recording', help='Base path to recording (without _poses.txt or _clouds.bin)')
    parser.add_argument('--format', choices=['ply', 'pcd'], default='ply', help='Output format (default: ply)')
    
    args = parser.parse_args()
    
    base_path = Path(args.recording)
    pose_file = Path(str(base_path) + "_poses.txt")
    cloud_file = Path(str(base_path) + "_clouds.bin")
    
    if args.format == 'ply':
        output_file = Path(str(base_path) + ".ply")
    else:
        output_file = Path(str(base_path) + ".pcd")
    
    if not pose_file.exists():
        print(f"Error: Pose file not found: {pose_file}")
        sys.exit(1)
    
    if not cloud_file.exists():
        print(f"Error: Cloud file not found: {cloud_file}")
        sys.exit(1)
    
    print(f"Reading poses from {pose_file}...")
    poses = read_poses(pose_file)
    print(f"Loaded {len(poses)} poses")
    
    print(f"Reading and transforming point clouds from {cloud_file}...")
    points = read_clouds(cloud_file, poses)
    
    if len(points) == 0:
        print("Error: No points found!")
        sys.exit(1)
    
    print(f"Total points: {len(points)}")
    print(f"Writing {args.format.upper()} file to {output_file}...")
    
    if args.format == 'ply':
        write_ply(points, output_file)
    else:
        write_pcd(points, output_file)
    
    print("\nDone! You can now open the file in:")
    print("  - CloudCompare: https://www.cloudcompare.org/")
    print("  - MeshLab: https://www.meshlab.net/")
    print("  - Blender (PLY only)")
    print("  - Online: https://www.creators3d.com/online-viewer")

if __name__ == "__main__":
    main()
