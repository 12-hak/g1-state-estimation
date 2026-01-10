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

        for i in range(-grid_range, grid_range + 1):
            for j in range(-grid_range, grid_range + 1):
                tile = spec.worldbody.add_geom()
                tile.type = mujoco.mjtGeom.mjGEOM_BOX
                tile.size = [tile_size / 2, tile_size / 2, 0.001]
                tile.pos = [i * tile_size, j * tile_size, -0.001]
                tile.contype = 0
                tile.conaffinity = 0
                # Checkerboard pattern
                if (i + j) % 2 == 0:
                    tile.material = "floor_light"
                else:
                    tile.material = "floor_dark"

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
        self.state_received = False
        self.last_state_time = 0
        self.state_lock = threading.Lock()

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
                data, addr = self.sock.recvfrom(4096)
                self._parse_packet(data)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Receive error: {e}")

    def _parse_packet(self, data: bytes):
        """Parse received UDP packet (supports v1, v2, and v3 formats)."""
        if len(data) < PACKET_SIZE_V1:
            return

        try:
            # Check magic bytes to determine version
            magic = data[:4]

            # Handle MAP packet (G1M1)
            if magic == b"G1M1" and len(data) >= 8:
                _, point_count = struct.unpack("<4sI", data[:8])
                if len(data) >= 8 + point_count * 12:
                    points_data = data[8 : 8 + point_count * 12]
                    points = struct.unpack(f"<{point_count * 3}f", points_data)
                    map_cloud = np.array(points).reshape(-1, 3)
                    with self.state_lock:
                        self.map_cloud = map_cloud
                        print(f"[Visualizer] Received reference map with {point_count} points.")
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

                # Parse point cloud if present
                point_cloud = np.empty((0, 3))
                if point_count > 0 and len(data) >= PACKET_SIZE_V4_BASE + point_count * 12:
                    points_data = data[PACKET_SIZE_V4_BASE : PACKET_SIZE_V4_BASE + point_count * 12]
                    points = struct.unpack(f"<{point_count * 3}f", points_data)
                    point_cloud = np.array(points).reshape(-1, 3)

                with self.state_lock:
                    self.position_raw = np.array(position_raw)
                    self.position = np.array(position_icp)  # Use ICP as primary
                    self.velocity = np.array(velocity)
                    self.joint_positions = np.array(joint_pos)
                    self.joint_velocities = np.array(joint_vel)
                    self.imu_quaternion = np.array(imu_quat)
                    self.imu_gyro = np.array(imu_gyro)
                    self.point_cloud = point_cloud
                    self.last_state_time = time.time()

                    if not self.state_received:
                        print(f"Receiving robot state (v4 with dual positioning and {point_count} LiDAR points)!")
                        self.state_received = True

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
            # Z will be computed by _ground_robot based on joint configuration
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
        """Render LiDAR point cloud and position markers."""
        # Reset geom counter for custom rendering
        viewer.user_scn.ngeom = 0
        
        with self.state_lock:
            # Only render if we have V4 data (point_cloud exists)
            # This prevents slowdown when using V3 packets
            if not hasattr(self, 'point_cloud'):
                return
                
            # Render LiDAR point cloud as small red dots (limit to 50 for performance)
            if len(self.point_cloud) > 0:
                max_points = min(50, len(self.point_cloud))
                for i in range(max_points):
                    point = self.point_cloud[i]
                    # Transform point from robot frame to world frame
                    world_point = self.position + point
                    
                    if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
                        break
                        
                    mujoco.mjv_initGeom(
                        viewer.user_scn.geoms[viewer.user_scn.ngeom],
                        mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([0.01, 0, 0], dtype=np.float64),  # tiny size
                        np.array(world_point, dtype=np.float64),
                        np.eye(3, dtype=np.float64).flatten(),
                        np.array([1, 0, 0, 0.8], dtype=np.float32)  # red
                    )
                    viewer.user_scn.ngeom += 1
            
            # Render Reference Map points (White)
            if hasattr(self, 'map_cloud') and self.map_cloud.size > 0:
                for i in range(min(100, len(self.map_cloud))):
                    point = self.map_cloud[i]
                    if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
                        break
                    mujoco.mjv_initGeom(
                        viewer.user_scn.geoms[viewer.user_scn.ngeom],
                        mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([0.015, 0, 0], dtype=np.float64),
                        np.array(point, dtype=np.float64),
                        np.eye(3, dtype=np.float64).flatten(),
                        np.array([1, 1, 1, 0.5], dtype=np.float32)  # white, semi-transparent
                    )
                    viewer.user_scn.ngeom += 1
            
            # Render position markers (smaller spheres)
            # Blue sphere for raw leg odometry
            if viewer.user_scn.ngeom < viewer.user_scn.maxgeom:
                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[viewer.user_scn.ngeom],
                    mujoco.mjtGeom.mjGEOM_SPHERE,
                    np.array([0.05, 0, 0], dtype=np.float64),  # smaller size
                    np.array([self.position_raw[0], self.position_raw[1], 0.05], dtype=np.float64),
                    np.eye(3, dtype=np.float64).flatten(),
                    np.array([0, 0, 1, 0.6], dtype=np.float32)  # blue, semi-transparent
                )
                viewer.user_scn.ngeom += 1
            
            # Green sphere for ICP-corrected position
            if viewer.user_scn.ngeom < viewer.user_scn.maxgeom:
                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[viewer.user_scn.ngeom],
                    mujoco.mjtGeom.mjGEOM_SPHERE,
                    np.array([0.05, 0, 0], dtype=np.float64),  # smaller size
                    np.array([self.position[0], self.position[1], 0.05], dtype=np.float64),
                    np.eye(3, dtype=np.float64).flatten(),
                    np.array([0, 1, 0, 0.6], dtype=np.float32)  # green, semi-transparent
                )
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

    args = parser.parse_args()

    if args.demo:
        print("Running in demo mode (no network connection)")
    else:
        print("Waiting for robot state from network...")
        print("Make sure g1_state_publisher.py is running on the G1")

    visualizer = G1NetworkVisualizer(demo_mode=args.demo)
    visualizer.run()


if __name__ == "__main__":
    main()
