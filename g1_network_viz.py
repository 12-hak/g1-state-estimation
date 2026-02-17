#!/usr/bin/env python3
"""
G1 Real-Time Visualization via Network - Run this on Windows PC

Receives robot state from the G1 over UDP and visualizes in MuJoCo.
The G1 must be running g1_state_publisher.py.

Usage:
    python g1_network_viz.py [--demo]

The script listens on UDP port 9870 for state data from the G1.
"""

import argparse
import socket
import struct
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


# UDP port (must match publisher)
UDP_PORT = 9870

# Packet format v4 with dual positioning and LiDAR data:
# - Magic bytes (4): "G1S4"
# - Timestamp (8): double
# - Position RAW/Leg Odom (3 * 4 = 12): floats (x, y, z)
# - Position ICP-Corrected (3 * 4 = 12): floats (x, y, z)
# - Velocity (2 * 4 = 8): floats (vx, vy)
# - Joint positions (29 * 4 = 116): floats
# - Joint velocities (29 * 4 = 116): floats
# - IMU quaternion (4 * 4 = 16): floats (w, x, y, z)
# - IMU gyro (3 * 4 = 12): floats
# - Point cloud count (4): uint32
# - Point cloud data (N * 12): N points, each 3 floats (x,y,z)
PACKET_SIZE_V4_BASE = 4 + 8 + 12 + 12 + 8 + 116 + 116 + 16 + 12 + 4  # 308 bytes + point cloud
PACKET_FORMAT_V4_HEADER = "<4sd3f3f2f29f29f4f3fI"  # Header without point cloud

# Also support v3 for backwards compatibility
PACKET_SIZE_V3 = 4 + 8 + 12 + 8 + 116 + 116 + 16 + 12  # 292 bytes
PACKET_FORMAT_V3 = "<4sd3f2f29f29f4f3f"
# Also support v2 for backwards compatibility
PACKET_SIZE_V2 = 4 + 8 + 12 + 116 + 116 + 16 + 12  # 284 bytes
PACKET_FORMAT_V2 = "<4sd3f29f29f4f3f"
# Also support old format for backwards compatibility
PACKET_SIZE_V1 = 4 + 8 + 116 + 116 + 16 + 12  # 272 bytes
PACKET_FORMAT_V1 = "<4sd29f29f4f3f"


class G1JointIndex:
    """Motor indices in the Unitree G1 (29 DoF)."""

    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28


# Mapping from G1 motor index to MuJoCo joint name
G1_MOTOR_TO_MUJOCO_JOINT = {
    G1JointIndex.LeftHipPitch: "left_hip_pitch_joint",
    G1JointIndex.LeftHipRoll: "left_hip_roll_joint",
    G1JointIndex.LeftHipYaw: "left_hip_yaw_joint",
    G1JointIndex.LeftKnee: "left_knee_joint",
    G1JointIndex.LeftAnklePitch: "left_ankle_pitch_joint",
    G1JointIndex.LeftAnkleRoll: "left_ankle_roll_joint",
    G1JointIndex.RightHipPitch: "right_hip_pitch_joint",
    G1JointIndex.RightHipRoll: "right_hip_roll_joint",
    G1JointIndex.RightHipYaw: "right_hip_yaw_joint",
    G1JointIndex.RightKnee: "right_knee_joint",
    G1JointIndex.RightAnklePitch: "right_ankle_pitch_joint",
    G1JointIndex.RightAnkleRoll: "right_ankle_roll_joint",
    G1JointIndex.WaistYaw: "waist_yaw_joint",
    G1JointIndex.WaistRoll: "waist_roll_joint",
    G1JointIndex.WaistPitch: "waist_pitch_joint",
    G1JointIndex.LeftShoulderPitch: "left_shoulder_pitch_joint",
    G1JointIndex.LeftShoulderRoll: "left_shoulder_roll_joint",
    G1JointIndex.LeftShoulderYaw: "left_shoulder_yaw_joint",
    G1JointIndex.LeftElbow: "left_elbow_joint",
    G1JointIndex.LeftWristRoll: "left_wrist_roll_joint",
    G1JointIndex.LeftWristPitch: "left_wrist_pitch_joint",
    G1JointIndex.LeftWristYaw: "left_wrist_yaw_joint",
    G1JointIndex.RightShoulderPitch: "right_shoulder_pitch_joint",
    G1JointIndex.RightShoulderRoll: "right_shoulder_roll_joint",
    G1JointIndex.RightShoulderYaw: "right_shoulder_yaw_joint",
    G1JointIndex.RightElbow: "right_elbow_joint",
    G1JointIndex.RightWristRoll: "right_wrist_roll_joint",
    G1JointIndex.RightWristPitch: "right_wrist_pitch_joint",
    G1JointIndex.RightWristYaw: "right_wrist_yaw_joint",
}


class G1NetworkVisualizer:
    """Visualizes G1 robot state received over UDP."""

    def __init__(self, demo_mode: bool = False):
        self.demo_mode = demo_mode

        # Load MuJoCo model
        xml_path = (
            Path(__file__).parent
            / "unitree_mujoco"
            / "unitree_robots"
            / "g1"
            / "scene.xml"
        )
        if not xml_path.exists():
            raise FileNotFoundError(f"G1 model not found at {xml_path}")

        print(f"Loading MuJoCo model from {xml_path}")

        # Load model via spec so we can add ground plane
        spec = mujoco.MjSpec.from_file(str(xml_path))

        # Create checkerboard floor using multiple colored geoms
        # Add two materials - light and dark
        light_mat = spec.add_material()
        light_mat.name = "floor_light"
        light_mat.rgba = [0.6, 0.65, 0.7, 1.0]  # Light gray

        dark_mat = spec.add_material()
        dark_mat.name = "floor_dark"
        dark_mat.rgba = [0.25, 0.28, 0.35, 1.0]  # Dark gray

        # Create checkerboard grid of boxes
        tile_size = 0.5  # 0.5m tiles
        grid_range = 20  # +/- 20 tiles = 40x40 = 20m x 20m area
        
        # Create checkerboard grid of boxes (Subtle for context)
        tile_size = 1.0  # 1m tiles
        grid_range = 10  # 20m x 20m area
        
        for i in range(-grid_range, grid_range):
            for j in range(-grid_range, grid_range):
                tile = spec.worldbody.add_geom()
                tile.type = mujoco.mjtGeom.mjGEOM_BOX
                tile.size = [tile_size / 2, tile_size / 2, 0.001]
                tile.pos = [i * tile_size + tile_size/2, j * tile_size + tile_size/2, -0.001]
                tile.contype = 0
                tile.conaffinity = 0
                tile.rgba = [0.15, 0.15, 0.18, 1.0] if (i + j) % 2 == 0 else [0.12, 0.12, 0.15, 1.0]

        # Compile the model
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)

        # Build joint name to qpos index mapping
        self.joint_name_to_qpos_idx = {}
        for i in range(self.model.njnt):
            jnt_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if jnt_name and jnt_name != "floating_base_joint":
                qpos_adr = self.model.jnt_qposadr[i]
                self.joint_name_to_qpos_idx[jnt_name] = qpos_adr

        # Build motor index to qpos index mapping
        self.motor_to_qpos = {}
        for motor_idx, joint_name in G1_MOTOR_TO_MUJOCO_JOINT.items():
            if joint_name in self.joint_name_to_qpos_idx:
                self.motor_to_qpos[motor_idx] = self.joint_name_to_qpos_idx[joint_name]

        print(f"Mapped {len(self.motor_to_qpos)} joints from G1 to MuJoCo")

        # State from network
        self.position = np.array([0.0, 0.0, 0.793])  # ICP-corrected position
        self.position_raw = np.array([0.0, 0.0, 0.793])  # Raw leg odometry
        self.velocity = np.zeros(2)  # vx, vy from odometry
        self.joint_positions = np.zeros(29)
        self.joint_velocities = np.zeros(29)
        self.imu_quaternion = np.array([1, 0, 0, 0])  # w, x, y, z (identity)
        self.imu_gyro = np.zeros(3)
        self.point_cloud = np.empty((0, 3))  # LiDAR point cloud
        self.map_cloud = np.empty((0, 3))  # Map point cloud (from G1M1 packets)
        self.map_point_ages = {}  # Track when each map point was last seen
        self.state_received = False
        self.last_state_time = 0
        self.state_lock = threading.Lock()
        
        # Scan quality tracking for visualization
        self.last_good_scan_time = 0
        self.last_scan_sequence = 0
        self.scan_sequence_counter = 0
        self.last_map_packet_time = 0  # Last time we received a G1M1 snapshot
        
        # Accumulate scans over time to build complete scan
        self.scan_buffer = []  # List of (time, points) tuples
        self.scan_buffer_max_age = 0.5  # Accumulate scans over 0.5 seconds
        
        # Robot path tracking
        self.robot_path = []  # List of positions the robot has visited
        self.last_path_update = 0
        self.path_update_interval = 0.1  # Add path point every 100ms
        
        # Persistent wall tracking
        self.wall_segments = {}  # Dict of wall segments: {grid_key: (point, last_seen_time, confidence)}
        self.wall_timeout = 600.0  # Walls persist for 10 minutes (keep points you don't see)
        
        # Rendering settings
        self.wall_height = 1.5  # Wall marker height in meters
        self.post_radius = 0.03  # Post radius
        self.max_render_points = 2500  # Increased since we're rendering two layers
        self.show_walls = True
        self.show_breadcrumbs = False
        self._debug_first_map = True  # Print debug on first map data
        self._debug_first_render = True  # Print debug on first render

        # Find base body for active scan transformation
        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        if self.base_body_id < 0:
            self.base_body_id = 1 # Fallback to first child of world

        # UDP receiver
        if not demo_mode:
            self._start_udp_receiver()

    def _start_udp_receiver(self):
        """Start background thread to receive UDP packets."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", UDP_PORT))
        self.sock.settimeout(0.1)  # Non-blocking with timeout

        self.running = True
        self.receiver_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receiver_thread.start()

        print(f"Listening for robot state on UDP port {UDP_PORT}...")

    def _receive_loop(self):
        """Background thread to receive UDP packets."""
        while self.running:
            try:
                # Buffer increased to 1MB to handle large PointCloud/Map packets
                data, addr = self.sock.recvfrom(1024 * 1024) 
                self._parse_packet(data)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Receive error: {e}")
    
    def _is_good_scan(self, points: np.ndarray) -> bool:
        """
        Check if a scan is good quality.
        Filters out scans with:
        - Too few points (small response)
        - Swirl patterns (circular clustering)
        - Points too clustered (not enough spread)
        """
        if points.size == 0:
            return False
        
        # Extract 2D points (x, y)
        points_2d = points[:, :2]
        n_points = len(points_2d)
        
        # 1. Minimum point count check (relaxed for accumulated scans)
        MIN_POINTS = 30  # Require at least 30 points (was 50, but we accumulate now)
        if n_points < MIN_POINTS:
            return False
        
        # 2. Check for sufficient spread (not all points in one place)
        if n_points > 0:
            center = np.mean(points_2d, axis=0)
            distances = np.linalg.norm(points_2d - center, axis=1)
            max_dist = np.max(distances)
            mean_dist = np.mean(distances)
            
            # Require reasonable spread (relaxed for smaller scans)
            # At least 0.3m max distance, 0.15m mean (was 0.5m/0.2m)
            if max_dist < 0.3 or mean_dist < 0.15:
                return False
        
        # 3. Swirl detection: Check if points form a circular pattern
        # Swirls typically have points clustered in a ring with low variance in distance from center
        if n_points > 10:
            # Calculate angular distribution
            angles = np.arctan2(points_2d[:, 1] - center[1], points_2d[:, 0] - center[0])
            # Normalize angles to [0, 2π]
            angles = (angles + 2 * np.pi) % (2 * np.pi)
            angles_sorted = np.sort(angles)
            
            # Check angular spread - swirls have points distributed around circle
            # but we want to detect if they're TOO evenly distributed (swirl pattern)
            angle_diffs = np.diff(angles_sorted)
            angle_diffs = np.append(angle_diffs, 2 * np.pi - angles_sorted[-1] + angles_sorted[0])
            
            # Swirl pattern: points evenly spaced in angle with similar distances
            # Check coefficient of variation of distances (low = similar distances = potential swirl)
            if len(distances) > 5:
                dist_std = np.std(distances)
                dist_mean = np.mean(distances)
                if dist_mean > 0:
                    dist_cv = dist_std / dist_mean
                    # If distances are very similar (CV < 0.25) AND angles are evenly spread, likely a swirl
                    # Made stricter to avoid false positives
                    angle_std = np.std(angle_diffs)
                    expected_angle_diff = 2 * np.pi / n_points
                    if dist_cv < 0.25 and angle_std < expected_angle_diff * 0.4:
                        return False  # Likely a swirl
        
        # 4. Check for reasonable point density (not too sparse)
        # Calculate nearest neighbor distances using numpy only
        if n_points > 20:
            # Sample a subset for performance
            sample_size = min(100, n_points)
            indices = np.random.choice(n_points, sample_size, replace=False)
            sample_points = points_2d[indices]
            
            # Find minimum distances (excluding self) using numpy
            min_dists = []
            for i, pt in enumerate(sample_points):
                dists = np.linalg.norm(sample_points - pt, axis=1)
                dists[i] = np.inf  # Exclude self
                min_dists.append(np.min(dists))
            
            mean_min_dist = np.mean(min_dists)
            
            # If points are too sparse (mean min distance > 1m), might be a bad scan
            if mean_min_dist > 1.0:
                return False
        
        # All checks passed - this is a good scan
        return True

    def _parse_packet(self, data: bytes):
        """Parse received UDP packet (supports v1, v2, and v3 formats)."""
        if len(data) < PACKET_SIZE_V1:
            return

        try:
            # Check magic bytes to determine version
            magic = data[:4]

            # MAP packet (G1M1): treat as the primary visualization scan.
            # After fixes in g1_localization, this should be an IMU-leveled horizontal slice
            # already in world coordinates (z ~= 0).
            if magic == b"G1M1" and len(data) >= 8:
                _, point_count = struct.unpack("<4sI", data[:8])
                if len(data) >= 8 + point_count * 12:
                    points_data = data[8 : 8 + point_count * 12]
                    points = struct.unpack(f"<{point_count * 3}f", points_data)
                    new_points = np.array(points).reshape(-1, 3)

                    current_time = time.time()
                    with self.state_lock:
                        if self._debug_first_map:
                            print(f"[DEBUG] First G1M1 packet: {len(new_points)} points")
                            if len(new_points) > 0:
                                print(f"[DEBUG]   Point range X: {new_points[:,0].min():.2f} to {new_points[:,0].max():.2f}")
                                print(f"[DEBUG]   Point range Y: {new_points[:,1].min():.2f} to {new_points[:,1].max():.2f}")
                                print(f"[DEBUG]   Point range Z: {new_points[:,2].min():.2f} to {new_points[:,2].max():.2f}")
                            self._debug_first_map = False
                        # PERSISTENT WALLS: Update wall segments with hysteresis
                        if new_points.size > 0:
                            robot_xy = self.position[:2]
                            # 1. Update/Add segments from the map packet
                            hit_keys = set()
                            for pt in new_points:
                                # Deduplicate robot's own body (Body Shadow Filter)
                                if np.linalg.norm(pt[:2] - robot_xy) < 0.35: continue
                                
                                key = (round(pt[0] / 0.05), round(pt[1] / 0.05))
                                hit_keys.add(key)
                                
                                if key in self.wall_segments:
                                    old_pt, old_time, hits, misses = self.wall_segments[key]
                                    # G1M1 points are very high confidence (SLAM output)
                                    self.wall_segments[key] = (pt, current_time, min(hits + 5, 50), 0)
                                else:
                                    self.wall_segments[key] = (pt, current_time, 5, 0)
                            
                            # 2. Maintenance: Rebuild Map Cloud
                            if not hasattr(self, '_last_map_maintenance'): self._last_map_maintenance = 0
                            if current_time - self._last_map_maintenance > 0.1:
                                # Remove points that are truly old (10 min) OR have too many misses
                                expired_keys = [k for k, v in self.wall_segments.items() 
                                               if (current_time - v[1] > self.wall_timeout) or (v[3] > 15)]
                                for k in expired_keys: del self.wall_segments[k]
                                
                                # Rebuild
                                self.map_cloud_data = [v for v in self.wall_segments.values() if v[2] >= 5]
                                if self.map_cloud_data:
                                    self.map_cloud = np.array([v[0] for v in self.map_cloud_data])
                                else:
                                    self.map_cloud = np.empty((0, 3))
                                self._last_map_maintenance = current_time

                            self.last_good_scan_time = current_time
                        self.last_map_packet_time = current_time

                return

            if magic == b"G1S4" and len(data) >= PACKET_SIZE_V4_BASE:
                # V4 format with dual positioning and point cloud
                unpacked = struct.unpack(PACKET_FORMAT_V4_HEADER, data[:PACKET_SIZE_V4_BASE])
                # timestamp = unpacked[1]  # unused
                position_raw = unpacked[2:5]  # 3 floats (x, y, z) - leg odometry
                position_icp = unpacked[5:8]  # 3 floats (x, y, z) - ICP corrected
                velocity = unpacked[8:10]  # 2 floats (vx, vy)
                joint_pos = unpacked[10:39]  # 29 floats
                joint_vel = unpacked[39:68]  # 29 floats
                imu_quat = unpacked[68:72]  # 4 floats (w, x, y, z)
                imu_gyro = unpacked[72:75]  # 3 floats
                point_count = unpacked[75]  # uint32

                # Parse point cloud if present (raw LiDAR scan in robot frame)
                point_cloud = np.empty((0, 3))
                if point_count > 0 and len(data) >= PACKET_SIZE_V4_BASE + point_count * 12:
                    points_data = data[PACKET_SIZE_V4_BASE : PACKET_SIZE_V4_BASE + point_count * 12]
                    points = struct.unpack(f"<{point_count * 3}f", points_data)
                    point_cloud = np.array(points).reshape(-1, 3)

                current_time = time.time()
                with self.state_lock:
                    # Update robot state
                    self.position = np.array(position_icp)
                    self.position_raw = np.array(position_raw)
                    self.velocity = np.array(velocity)
                    self.joint_positions = np.array(joint_pos)
                    self.joint_velocities = np.array(joint_vel)
                    self.imu_quaternion = np.array(imu_quat)
                    self.imu_gyro = np.array(imu_gyro)
                    self.point_cloud = point_cloud
                    self.last_state_time = current_time
                    self.state_received = True

                    # --- STICKY NEIGHBOR MAPPING ---
                    if point_cloud.size > 1:
                        # 1. Connectivity Check: Only points with a neighbor within 15cm are considered "Solid"
                        neighbor_threshold_sq = 0.15**2 
                        is_solid = np.zeros(len(point_cloud), dtype=bool)
                        
                        for i in range(len(point_cloud)):
                            p1 = point_cloud[i]
                            # Check neighbors in a small window
                            for j in range(max(0, i-5), min(len(point_cloud), i+6)):
                                if i == j: continue
                                if np.sum((p1 - point_cloud[j])**2) < neighbor_threshold_sq:
                                    is_solid[i] = True
                                    break
                        
                        solid_points = point_cloud[is_solid]
                        
                        # 2. Transform to World Frame
                        w, x, y, z = imu_quat
                        R = np.array([
                            [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
                            [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                            [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]
                        ])
                        world_pts = (solid_points @ R.T) + self.position
                        robot_xy = self.position[:2]
                        
                        # 3. PROBABILISTIC MAP UPDATE
                        # Standard Log-Odds Approach:
                        # - Hit: Increase confidence (Evidence of obstacle)
                        # - Miss: Decrease confidence (Evidence of free space)
                        # - Decay: None (Static world assumption)
                        
                        # A. Process HITS
                        hit_keys = set()
                        valid_mask = (world_pts[:, 2] > 0.15) & (world_pts[:, 2] < 2.0)
                        
                        HIT_BONUS = 2.0
                        MAX_CONFIDENCE = 10.0
                        
                        for pt in world_pts[valid_mask]:
                            key = (round(pt[0] / 0.05), round(pt[1] / 0.05))
                            hit_keys.add(key)
                            
                            if key in self.wall_segments:
                                mean_pt, last_time, confidence, _ = self.wall_segments[key]
                                # Boost confidence
                                new_conf = min(confidence + HIT_BONUS, MAX_CONFIDENCE)
                                # Update position average with slight bias to new data
                                new_pt = (mean_pt * 0.7 + pt * 0.3)
                                self.wall_segments[key] = (new_pt, current_time, new_conf, 0)
                            else:
                                # New discovery starts at 5.0 (Instant Render)
                                self.wall_segments[key] = (pt, current_time, 5.0, 0)

                        # B. Process MISSES (Visibility Clearing)
                        # Only check cells that are:
                        # 1. Within range of the lidar (4.0m)
                        # 2. Within the angular field-of-view of this packet
                        # 3. NOT hit by this packet
                        
                        scan_angles = np.arctan2(point_cloud[:, 1], point_cloud[:, 0])
                        angle_min, angle_max = np.min(scan_angles), np.max(scan_angles)
                        
                        MISS_PENALTY = 0.5
                        
                        # Optimization: Only iterate if we have a valid wedge
                        if angle_max - angle_min < np.pi: # Sanity check for wrapped scans
                            keys_to_check = [k for k, v in self.wall_segments.items() 
                                           if np.linalg.norm(v[0][:2] - robot_xy) < 4.0]
                            
                            for k in keys_to_check:
                                if k in hit_keys: continue
                                
                                pt_world = self.wall_segments[k][0]
                                pt_rel = pt_world[:2] - robot_xy
                                # Rotate to robot frame
                                pt_robot = np.dot(R[:2, :2].T, pt_rel)
                                pt_angle = np.arctan2(pt_robot[1], pt_robot[0])
                                
                                if angle_min <= pt_angle <= angle_max:
                                    mean_pt, last_time, confidence, misses = self.wall_segments[k]
                                    new_conf = confidence - MISS_PENALTY
                                    
                                    if new_conf <= 0.0:
                                        del self.wall_segments[k]
                                    else:
                                        self.wall_segments[k] = (mean_pt, last_time, new_conf, misses + 1)
                        
                        # 4. Map Rebuild (Throttle to 10Hz)
                        if not hasattr(self, '_last_map_maintenance'): self._last_map_maintenance = 0
                        if current_time - self._last_map_maintenance > 0.1:
                            # Only render points with High Confidence (> 5.0)
                            # This creates a "Solid Core" map
                            active_map = [v for v in self.wall_segments.values() if v[2] > 4.0]
                            if active_map:
                                self.map_cloud_data = active_map
                                self.map_cloud = np.array([v[0] for v in active_map])
                            else:
                                self.map_cloud = np.empty((0, 3))
                            self._last_map_maintenance = current_time

            elif magic == b"G1S3" and len(data) >= PACKET_SIZE_V3:
                # V3 format with position and velocity
                unpacked = struct.unpack(PACKET_FORMAT_V3, data[:PACKET_SIZE_V3])
                # timestamp = unpacked[1]  # unused
                position = unpacked[2:5]  # 3 floats (x, y, z)
                velocity = unpacked[5:7]  # 2 floats (vx, vy)
                joint_pos = unpacked[7:36]  # 29 floats
                joint_vel = unpacked[36:65]  # 29 floats
                imu_quat = unpacked[65:69]  # 4 floats (w, x, y, z)
                imu_gyro = unpacked[69:72]  # 3 floats

                with self.state_lock:
                    self.position = np.array(position)
                    self.velocity = np.array(velocity)
                    self.joint_positions = np.array(joint_pos)
                    self.joint_velocities = np.array(joint_vel)
                    self.imu_quaternion = np.array(imu_quat)
                    self.imu_gyro = np.array(imu_gyro)
                    self.last_state_time = time.time()

                    if not self.state_received:
                        print("Receiving robot state (v3 with position and velocity)!")
                        self.state_received = True

            elif magic == b"G1S2" and len(data) >= PACKET_SIZE_V2:
                # V2 format with position
                unpacked = struct.unpack(PACKET_FORMAT_V2, data[:PACKET_SIZE_V2])
                # timestamp = unpacked[1]  # unused
                position = unpacked[2:5]  # 3 floats (x, y, z)
                joint_pos = unpacked[5:34]  # 29 floats
                joint_vel = unpacked[34:63]  # 29 floats
                imu_quat = unpacked[63:67]  # 4 floats (w, x, y, z)
                imu_gyro = unpacked[67:70]  # 3 floats

                with self.state_lock:
                    self.position = np.array(position)
                    self.velocity = np.zeros(2)  # No velocity in v2
                    self.joint_positions = np.array(joint_pos)
                    self.joint_velocities = np.array(joint_vel)
                    self.imu_quaternion = np.array(imu_quat)
                    self.imu_gyro = np.array(imu_gyro)
                    self.last_state_time = time.time()

                    if not self.state_received:
                        print("Receiving robot state (v2 with position)!")
                        self.state_received = True

            elif magic == b"G1ST" and len(data) >= PACKET_SIZE_V1:
                # V1 format without position (backwards compatible)
                unpacked = struct.unpack(PACKET_FORMAT_V1, data[:PACKET_SIZE_V1])
                joint_pos = unpacked[2:31]  # 29 floats
                joint_vel = unpacked[31:60]  # 29 floats
                imu_quat = unpacked[60:64]  # 4 floats
                imu_gyro = unpacked[64:67]  # 3 floats

                with self.state_lock:
                    self.joint_positions = np.array(joint_pos)
                    self.joint_velocities = np.array(joint_vel)
                    self.imu_quaternion = np.array(imu_quat)
                    self.imu_gyro = np.array(imu_gyro)
                    self.last_state_time = time.time()

                    if not self.state_received:
                        print("Receiving robot state (v1)!")
                        self.state_received = True

        except struct.error as e:
            print(f"Packet parse error: {e}")

    def _ground_robot(self):
        """Adjust base height so feet stay on the ground."""
        self.data.qpos[2] = 2.0
        mujoco.mj_forward(self.model, self.data)

        lowest_z = None
        
        # Try foot sites first
        left_foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "left_foot")
        right_foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "right_foot")
        
        if left_foot_id >= 0 and right_foot_id >= 0:
            left_z = self.data.site_xpos[left_foot_id, 2]
            right_z = self.data.site_xpos[right_foot_id, 2]
            lowest_z = min(left_z, right_z) - 0.025
        else:
            # Scan all bodies to find feet/ankles (one-time setup)
            if not hasattr(self, '_foot_body_ids'):
                self._foot_body_ids = []
                for i in range(self.model.nbody):
                    name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
                    if name and any(x in name.lower() for x in ['ankle', 'foot', 'toe']):
                        self._foot_body_ids.append(i)
            
            # Use found foot bodies
            if self._foot_body_ids:
                for body_id in self._foot_body_ids:
                    z = self.data.xpos[body_id, 2]
                    if lowest_z is None or z < lowest_z:
                        lowest_z = z
                if lowest_z is not None:
                    lowest_z -= 0.05
        
        # Apply grounding
        if lowest_z is not None:
            self.data.qpos[2] -= lowest_z
        else:
            self.data.qpos[2] = 0.793

    def _update_from_network(self):
        """Update MuJoCo model from received network state."""
        with self.state_lock:
            # Update base position from state estimation (X, Y ONLY)
            # The robot is now centered EXACTLY on the converged (ICP) position.
            self.data.qpos[0] = self.position[0]
            self.data.qpos[1] = self.position[1]
            # Don't set qpos[2] here - let _ground_robot compute it

            # Update joint positions FIRST (before grounding)
            for motor_idx, qpos_idx in self.motor_to_qpos.items():
                self.data.qpos[qpos_idx] = self.joint_positions[motor_idx]

            # Update base orientation from IMU
            # Unitree IMU quaternion is [w, x, y, z] - same as MuJoCo
            self.data.qpos[3] = self.imu_quaternion[0]  # w
            self.data.qpos[4] = self.imu_quaternion[1]  # x
            self.data.qpos[5] = self.imu_quaternion[2]  # y
            self.data.qpos[6] = self.imu_quaternion[3]  # z
            
            # Track robot path
            current_time = time.time()
            if current_time - self.last_path_update > self.path_update_interval:
                self.robot_path.append(self.position[:2].copy())  # Store x, y
                self.last_path_update = current_time
                # Limit path length to last 1000 points
                if len(self.robot_path) > 1000:
                    self.robot_path.pop(0)

        # Ground the robot - compute FK and adjust height so feet touch ground
        # This MUST happen after joints are set but before we print/use the state
        self._ground_robot()

    def _demo_update(self, t: float):
        """Demo mode: generate synthetic joint movements."""
        for motor_idx, qpos_idx in self.motor_to_qpos.items():
            joint_name = G1_MOTOR_TO_MUJOCO_JOINT.get(motor_idx, "")
            if "elbow" in joint_name:
                self.data.qpos[qpos_idx] = 0.3 + 0.2 * np.sin(t * 2)
            elif "shoulder_pitch" in joint_name:
                self.data.qpos[qpos_idx] = 0.2 * np.sin(t * 1.5)
            elif "hip_pitch" in joint_name:
                self.data.qpos[qpos_idx] = -0.1 + 0.05 * np.sin(t)
            elif "knee" in joint_name:
                self.data.qpos[qpos_idx] = 0.3 + 0.1 * np.sin(t)
            elif "ankle_pitch" in joint_name:
                self.data.qpos[qpos_idx] = -0.2 - 0.05 * np.sin(t)

        # Set orientation and ground the robot
        self.data.qpos[3:7] = [1, 0, 0, 0]
        self._ground_robot()
    
    def _handle_keyboard(self, viewer):
        """Handle keyboard input for mode switching."""
        # Check for 'M' key press to cycle rendering modes
        # Note: MuJoCo viewer keyboard handling is limited in passive mode
        # This is a simplified approach - in practice, you might need to use
        # a different method to capture keyboard input
        pass  # Placeholder - MuJoCo passive viewer doesn't easily support custom key bindings

    def run(self):
        """Main visualization loop."""
        print("\nStarting MuJoCo visualization...")
        print("Press ESC or close the window to exit.\n")

        start_time = time.time()
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # Hide side panel UI menus - double-click on the viewer to toggle them back
            # Note: The viewer UI is controlled by the viewer itself, not easily hidden programmatically
            # Users can press Tab to hide/show the UI panels
            print("Tip: Press TAB to hide/show the side UI panels\n")
            
            while viewer.is_running():
                step_start = time.time()

                # Update from network or demo
                if self.demo_mode:
                    self._demo_update(time.time() - start_time)
                elif self.state_received:
                    self._update_from_network()
                else:
                    pass

                # Forward kinematics
                mujoco.mj_forward(self.model, self.data)
                
                # Render LiDAR point cloud and position markers
                self._render_debug_viz(viewer)
                
                viewer.sync()

                # Rate limiting (~50 Hz)
                elapsed = time.time() - step_start
                sleep_time = 0.02 - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        self.running = False
        print("Visualization ended.")
    
    def _render_debug_viz(self, viewer):
        """Render walls/obstacles as thin vertical posts and position markers."""
        # 1. Thread-safe snapshot of the state
        with self.state_lock:
            if not self.state_received: return
            viz_pos = self.position.copy()
            viz_quat = self.imu_quaternion.copy()
            
            # Snapshots of clouds
            active_raw = None
            if hasattr(self, 'point_cloud') and self.point_cloud.size > 0:
                active_raw = self.point_cloud.copy()
                
            map_pts = None
            map_meta = None
            if hasattr(self, 'map_cloud') and self.map_cloud.size > 0:
                map_pts = self.map_cloud.copy()
                map_meta = self.map_cloud_data[:] # Metadata (persistence)
        
        # 2. Prepare Scene
        viewer.user_scn.ngeom = 0
        if viewer.user_scn.maxgeom < 10000:
            viewer.user_scn.maxgeom = 10000
            viewer.user_scn.geoms = mujoco.MjvGeom(10000)
        max_geom = viewer.user_scn.maxgeom
        identity_rot = np.eye(3, dtype=np.float64).flatten()
        
        # Pre-calc robot 2D pos and rendering params
        robot_pos_2d = viz_pos[:2]
        height = self.wall_height
        half_height = height / 2.0
        radius = self.post_radius
        post_size = np.array([radius, radius, half_height], dtype=np.float64)

        # --- LAYER 1: PERSISTENT MAP (Solid Core) ---
        if self.show_walls and map_pts is not None:
            # We now only render points that are ALREADY filtered to > 4.0 confidence
            # So everything here is "Solid"
            
            # Simple downsample if too many points (limit to 10k)
            step = 1
            if len(map_pts) > 8000:
                step = 2
                
            dists = np.linalg.norm(map_pts[:, :2] - robot_pos_2d, axis=1)
            
            for i in range(0, len(map_pts), step):
                if viewer.user_scn.ngeom >= max_geom: break
                
                point = map_pts[i]
                confidence = map_meta[i][2] if i < len(map_meta) else 5.0
                
                # Alpha driven by confidence (4.0 to 10.0 mapping)
                # 5.0 = 0.5 alpha
                # 10.0 = 1.0 alpha
                alpha = min(confidence / 10.0, 1.0)
                
                # Color gradient (Orange for near, Purple for far)
                dist = dists[i]
                if dist < 2.5:
                    color = np.array([1.0, 0.4, 0.0, alpha], dtype=np.float32)
                else:
                    color = np.array([0.4, 0.0, 0.6, alpha * 0.8], dtype=np.float32)
                
                post_pos = np.array([point[0], point[1], half_height], dtype=np.float64)
                mujoco.mjv_initGeom(viewer.user_scn.geoms[viewer.user_scn.ngeom],
                                   mujoco.mjtGeom.mjGEOM_BOX, post_size, post_pos, identity_rot, color)
                viewer.user_scn.ngeom += 1

        # --- LAYER 2: ACTIVE SCAN (The High Refresh Cyan Slice) ---
        if self.show_walls and active_raw is not None:
            # Transform active scan slice to world frame using robot's current pose
            w, x, y, z = viz_quat
            R = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
                [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]
            ])
            active_world = (active_raw @ R.T) + viz_pos
            
            active_post_size = np.array([radius * 1.1, radius * 1.1, half_height], dtype=np.float64)
            active_color = np.array([0.0, 1.0, 1.0, 0.9], dtype=np.float32) # Bright Cyan
            
            for point in active_world:
                if viewer.user_scn.ngeom >= max_geom: break
                # Height filter for active scan visibility
                if point[2] < 0.15 or point[2] > 2.0: continue
                
                post_pos = np.array([point[0], point[1], half_height], dtype=np.float64)
                mujoco.mjv_initGeom(viewer.user_scn.geoms[viewer.user_scn.ngeom],
                                   mujoco.mjtGeom.mjGEOM_BOX, active_post_size, post_pos, identity_rot, active_color)
                viewer.user_scn.ngeom += 1

        # --- LAYER 4: ROBOT PATH (Breadcrumbs) ---
        if self.show_breadcrumbs and len(self.robot_path) > 1:
            # Downsample path for performance (render every 5th point)
            path_pts = self.robot_path[::5] 
            path_size = np.array([radius * 0.5, radius * 0.5, radius * 0.5], dtype=np.float64)
            
            for i, p_pos in enumerate(path_pts):
                if viewer.user_scn.ngeom >= max_geom: break
                
                # Fading effect for older crumbs
                age_ratio = (i / len(path_pts))
                path_color = np.array([1.0, 1.0, 1.0, 0.2 + 0.3 * age_ratio], dtype=np.float32) # Faded white
                
                mujoco.mjv_initGeom(viewer.user_scn.geoms[viewer.user_scn.ngeom],
                                   mujoco.mjtGeom.mjGEOM_SPHERE, path_size, np.array([p_pos[0], p_pos[1], 0.015]), identity_rot, path_color)
                viewer.user_scn.ngeom += 1

        # --- LAYER 5: ROBOT CONVERGED POSITION (Green Sphere) ---
        if viewer.user_scn.ngeom < max_geom:
            sphere_pos = np.array([viz_pos[0], viz_pos[1], 0.1], dtype=np.float64)
            sphere_size = np.array([0.1, 0.1, 0.1], dtype=np.float64)
            sphere_color = np.array([0.0, 1.0, 0.0, 0.8], dtype=np.float32) # Bright Green
            
            mujoco.mjv_initGeom(viewer.user_scn.geoms[viewer.user_scn.ngeom],
                               mujoco.mjtGeom.mjGEOM_SPHERE, sphere_size, sphere_pos, identity_rot, sphere_color)
            viewer.user_scn.ngeom += 1



def main():
    parser = argparse.ArgumentParser(
        description="Visualize G1 robot state from network in MuJoCo"
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Run in demo mode without network connection",
    )
    parser.add_argument(
        "--height",
        type=float,
        default=1.5,
        help="Wall marker height in meters (default: 1.5)",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=0.03,
        help="Post radius in meters (default: 0.03)",
    )
    parser.add_argument(
        "--no-walls",
        action="store_true",
        help="Disable wall/obstacle visualization",
    )
    parser.add_argument(
        "--breadcrumbs",
        action="store_true",
        help="Enable breadcrumb trail (spheres) for robot history",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=1500,
        help="Maximum points to render in map (default: 1500)",
    )

    args = parser.parse_args()

    if args.demo:
        print("Running in demo mode (no network connection)")
    else:
        print("Waiting for robot state from network...")
        print("Make sure g1_state_publisher.py is running on the G1")
    
    visualizer = G1NetworkVisualizer(demo_mode=args.demo)
    visualizer.wall_height = args.height
    visualizer.post_radius = args.radius
    visualizer.show_walls = not args.no_walls
    visualizer.show_breadcrumbs = args.breadcrumbs
    visualizer.max_render_points = args.max_points
    
    print(f"Wall markers: {'Enabled' if visualizer.show_walls else 'Disabled'}")
    print(f"Breadcrumbs: {'Enabled' if visualizer.show_breadcrumbs else 'Disabled'}")
    print(f"Max points: {visualizer.max_render_points}")
    visualizer.run()


if __name__ == "__main__":
    main()
