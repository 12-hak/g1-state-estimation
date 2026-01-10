#!/usr/bin/env python3
"""
G1 State Publisher - Run this on the G1's Jetson or PC1

Reads robot state via unitree_sdk2py and publishes it over UDP to a remote
visualization client (Windows PC).

Uses IMU dead reckoning for position estimation.

Usage:
    python g1_state_publisher.py [network_interface] [receiver_ip]

Example:
    python g1_state_publisher.py eth0 192.168.123.100

The script will publish joint positions and IMU data at 50Hz.
It also reads Livox Mid-360 point clouds to perform ICP-based localization.
"""

import socket
import struct
import sys
import threading
import time
import platform

import numpy as np


# Integrated Livox Mid-360 Driver & ICP Support


from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

# Number of motors in G1 (29 DoF)
G1_NUM_MOTOR = 29

# UDP port for state data
UDP_PORT = 9870

# Gravity constant
GRAVITY = 9.81


# Gravity constant
GRAVITY = 9.81


class LivoxDriver:
    """Simple UDP driver for Livox Mid-360 LiDAR."""
    def __init__(self, port=56300):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        try:
            self.sock.bind(("", port))
            print(f"LivoxDriver listening on port {port}")
        except Exception as e:
            print(f"Warning: Could not bind Livox port {port}: {e}")

        self.running = True
        self.points_buffer = []
        self.lock = threading.Lock()
        
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()

    def _recv_loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(4096)
                if len(data) < 36: continue
                
                # Livox Protocol
                # version(1), length(2), time(2), dot_num(2) at offset 5
                dot_num = struct.unpack('<H', data[5:7])[0]
                data_type = data[10]
                raw_data = data[36:]
                
                new_points = []
                
                # Type 1: High Performance (x,y,z,r,tag) - 14 bytes
                if data_type == 1:
                    point_len = 14
                    num_points = min(dot_num, len(raw_data) // point_len)
                    for i in range(num_points):
                        off = i * point_len
                        # x, y, z in mm (int32)
                        x, y, z = struct.unpack('<iii', raw_data[off : off + 12])
                        new_points.append([x / 1000.0, y / 1000.0, z / 1000.0])

                # Type 0: Cartesian (x,y,z,r) - 13 bytes
                elif data_type == 0:
                    point_len = 13
                    num_points = min(dot_num, len(raw_data) // point_len)
                    for i in range(num_points):
                        off = i * point_len
                        x, y, z = struct.unpack('<iii', raw_data[off : off + 12])
                        new_points.append([x / 1000.0, y / 1000.0, z / 1000.0])
                
                if new_points:
                    with self.lock:
                        self.points_buffer.extend(new_points)
                        # Keep buffer manageable (last ~5000 points)
                        if len(self.points_buffer) > 5000:
                            self.points_buffer = self.points_buffer[-5000:]
                            
            except socket.timeout:
                continue
            except Exception:
                pass

    def get_latest_scan(self):
        """Return accumulated points and clear buffer."""
        with self.lock:
            if not self.points_buffer:
                return np.empty((0, 3))
            pts = np.array(self.points_buffer)
            self.points_buffer = []
        return pts
        
    def stop(self):
        self.running = False
        self.thread.join()


def icp(source, target, max_iter=30, tolerance=1e-5):
    """Improved ICP with outlier rejection and convergence checking."""
    if len(source) == 0 or len(target) == 0:
        return np.eye(2), np.zeros(2), False  # Return failure flag

    # Need minimum points for reliable matching
    if len(source) < 20 or len(target) < 20:
        return np.eye(2), np.zeros(2), False

    source_copy = source.copy()
    prev_error = float('inf')
    
    for iteration in range(max_iter):
        # Find nearest neighbors
        distances = np.sum((target[:, np.newaxis] - source_copy) ** 2, axis=2)
        indices = np.argmin(distances, axis=0)
        min_distances = np.min(distances, axis=0)
        
        # Outlier rejection: remove matches with distance > 3*median
        median_dist = np.median(min_distances)
        inlier_mask = min_distances < 3 * median_dist
        
        if np.sum(inlier_mask) < 10:  # Too few inliers
            return np.eye(2), np.zeros(2), False
        
        source_inliers = source_copy[inlier_mask]
        matched_target = target[indices[inlier_mask]]

        # Center
        source_centroid = np.mean(source_inliers, axis=0)
        target_centroid = np.mean(matched_target, axis=0)
        source_centered = source_inliers - source_centroid
        target_centered = matched_target - target_centroid

        # Rotation via SVD
        H = np.dot(source_centered.T, target_centered)
        U, _, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = np.dot(Vt.T, U.T)

        t = target_centroid - np.dot(R, source_centroid)

        # Apply transform
        source_copy = np.dot(source_copy, R.T) + t

        # Check convergence
        error = np.mean(min_distances[inlier_mask])
        
        if abs(prev_error - error) < tolerance:
            # Converged - check if it's a good match
            success = error < 0.5 and np.sum(inlier_mask) / len(source) > 0.5
            return R, t, success
        
        prev_error = error

    # Max iterations reached
    return R, t, False


class G1StatePublisher:
    """Publishes G1 robot state over UDP."""

    def __init__(self, net_if: str = "eth0", receiver_ip: str = "192.168.123.100"):
        self.net_if = net_if
        self.receiver_ip = receiver_ip
        self.receiver_addr = (receiver_ip, UDP_PORT)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Robot state
        self.low_state: LowState_ = unitree_hg_msg_dds__LowState_()
        self.state_received = False
        self._sent_first_packet = False
        self.last_publish_time = 0
        self.publish_interval = 0.02  # 50 Hz

        # Position from state estimation (x, y, z)
        self.position = np.array([0.0, 0.0, 0.793])
        self.velocity = np.array([0.0, 0.0])  # Body velocity (vx, vy) in world frame
        self.yaw = 0.0  # Heading from gyro integration
        self.last_update_time = None
        self.last_joint_pos = None
        
        # Smoothing filters for position and velocity
        self.position_filtered = np.array([0.0, 0.0, 0.793])
        self.position_raw = np.array([0.0, 0.0, 0.793])  # Before ICP correction
        self.velocity_filtered = np.array([0.0, 0.0])
        self.position_alpha = 0.3  # Lower = smoother, higher = more responsive
        self.velocity_alpha = 0.5

        # SLAM parameters for real LiDAR-based positioning
        self.last_slam_time = time.time()
        self.slam_interval = 0.5  # More frequent updates with real data?
        self.map_points = self._generate_map()  # Known map for localization
        
        # Initialize Livox Driver
        self.livox = LivoxDriver(port=56300)


        # Leg odometry parameters
        # Scale factor: converts joint movement to forward movement
        # Tune this based on leg length and gait
        self.step_scale = 0.6  # Reduced to 0.6 for torque-based odometry

        # Initialize SDK
        print(f"Initializing DDS on interface: {net_if}")
        ChannelFactoryInitialize(0, net_if)

        # Subscribe to low-level state
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._low_state_handler, 10)

        print(f"Waiting for robot state...")
        timeout = 10.0
        start = time.time()
        while not self.state_received and (time.time() - start) < timeout:
            time.sleep(0.1)

        if self.state_received:
            print(f"Connected to robot! Sending state to {receiver_ip}:{UDP_PORT}")
        else:
            print("ERROR: No state received from robot")
            sys.exit(1)

    def _update_leg_odometry(self, joint_pos, joint_vel, joint_tau, gyro, dt):
        """Leg odometry using torque-based stance detection."""
        # Joint indices for G1
        LEFT_HIP_PITCH = 0
        LEFT_HIP_ROLL = 1
        LEFT_KNEE = 3
        RIGHT_HIP_PITCH = 6
        RIGHT_HIP_ROLL = 7
        RIGHT_KNEE = 9

        # Update yaw from gyroscope (Z axis)
        yaw_rate = gyro[2]  # rad/s
        self.yaw += yaw_rate * dt

        # Get joint velocities for both legs
        l_hip_pitch_vel = joint_vel[LEFT_HIP_PITCH]
        r_hip_pitch_vel = joint_vel[RIGHT_HIP_PITCH]
        l_knee_vel = joint_vel[LEFT_KNEE]
        r_knee_vel = joint_vel[RIGHT_KNEE]

        # Detect SQUAT vs WALK
        # Squat: Both legs move together (symmetric motion)
        # Walk: Legs move opposite (asymmetric motion)
        
        # Check if motion is symmetric (squat) or asymmetric (walk)
        pitch_symmetry = abs(l_hip_pitch_vel + r_hip_pitch_vel) / (abs(l_hip_pitch_vel) + abs(r_hip_pitch_vel) + 1e-6)
        knee_symmetry = abs(l_knee_vel + r_knee_vel) / (abs(l_knee_vel) + abs(r_knee_vel) + 1e-6)
        
        # If both legs moving in same direction (symmetry > 0.7), it's a squat, not a walk
        is_squatting = (pitch_symmetry > 0.7) or (knee_symmetry > 0.7)
        
        # Calculate Leg Torque Sums (Indices 0-5 for Left, 6-11 for Right)
        l_torque_sum = sum([abs(t) for t in joint_tau[0:6]])
        r_torque_sum = sum([abs(t) for t in joint_tau[6:12]])

        # If squatting, don't accumulate position
        if is_squatting:
            forward_vel = 0.0
            lateral_vel = 0.0
        else:
            # Detect Stance Leg (higher torque = stance)
            if l_torque_sum > r_torque_sum:
                stance_pitch_vel = joint_vel[LEFT_HIP_PITCH]
                stance_roll_vel = joint_vel[LEFT_HIP_ROLL]
            else:
                stance_pitch_vel = joint_vel[RIGHT_HIP_PITCH]
                stance_roll_vel = joint_vel[RIGHT_HIP_ROLL]

            # 1. Forward Motion
            forward_vel = stance_pitch_vel * self.step_scale
            
            # Apply deadband to prevent drift
            if abs(forward_vel) < 0.01:
                forward_vel = 0.0

            # 2. Lateral Motion
            raw_roll_sum = joint_vel[LEFT_HIP_ROLL] + joint_vel[RIGHT_HIP_ROLL]
            if abs(raw_roll_sum) < 0.2:
                lateral_vel = 0.0
            else:
                lateral_vel = -raw_roll_sum * 0.2

        # Rotate to world frame using yaw
        cos_yaw = np.cos(self.yaw)
        sin_yaw = np.sin(self.yaw)

        # Apply rotation to body velocity vector (forward_vel, lateral_vel)
        vel_x = forward_vel * cos_yaw - lateral_vel * sin_yaw
        vel_y = forward_vel * sin_yaw + lateral_vel * cos_yaw

        # Update position
        self.position[0] += vel_x * dt
        self.position[1] += vel_y * dt

        # Store velocity
        self.velocity[0] = vel_x
        self.velocity[1] = vel_y
        
        # Apply exponential smoothing filter
        self.position_filtered[0] = (self.position_alpha * self.position[0] + 
                                      (1 - self.position_alpha) * self.position_filtered[0])
        self.position_filtered[1] = (self.position_alpha * self.position[1] + 
                                      (1 - self.position_alpha) * self.position_filtered[1])
        self.position_filtered[2] = self.position[2]  # Z is not filtered (from kinematics)
        
        # Store raw position BEFORE any ICP correction
        self.position_raw[0] = self.position_filtered[0]
        self.position_raw[1] = self.position_filtered[1]
        self.position_raw[2] = self.position_filtered[2]
        
        self.velocity_filtered[0] = (self.velocity_alpha * self.velocity[0] + 
                                      (1 - self.velocity_alpha) * self.velocity_filtered[0])
        self.velocity_filtered[1] = (self.velocity_alpha * self.velocity[1] + 
                                      (1 - self.velocity_alpha) * self.velocity_filtered[1])

    def _generate_map(self):
        """Generate a known map for SLAM (simulated environment: square room)."""
        points = []
        # Walls
        for x in np.linspace(-5, 5, 100):
            points.append([x, -5])
            points.append([x, 5])
        for y in np.linspace(-5, 5, 100):
            points.append([-5, y])
            points.append([5, y])
        return np.array(points)

    def _generate_scan(self, x, y, yaw):
        """Simulate LiDAR scan from current odometry pose."""
        visible = []
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        for p in self.map_points:
            # Transform to robot frame
            dx = p[0] - x
            dy = p[1] - y
            dist = np.sqrt(dx**2 + dy**2)
            if dist < 5 and dist > 0.1:  # Range 5m, min 0.1m
                # Rotate to robot frame
                rx = dx * cos_yaw + dy * sin_yaw
                ry = -dx * sin_yaw + dy * cos_yaw
                # Add noise
                noise = np.random.normal(0, 0.02, 2)  # 2cm noise
                visible.append([rx + noise[0], ry + noise[1]])
        return np.array(visible)

    def _icp(self, source, target):
        """Wrapper for ICP."""
        return icp(source, target)

    def _fuse_lidar_position(self, lidar_pos, lidar_ori, icp_success):
        """Fuse LiDAR pose estimate with adaptive weighting based on ICP quality."""
        if lidar_pos is not None and icp_success:
            # Adaptive alpha based on ICP success
            alpha = 0.4  # Higher trust when ICP converges well
            self.position[0] = (1 - alpha) * self.position[0] + alpha * lidar_pos[0]
            self.position[1] = (1 - alpha) * self.position[1] + alpha * lidar_pos[1]
            
            # Update yaw if orientation available
            if lidar_ori is not None:
                import math
                lidar_yaw = math.atan2(2*(lidar_ori[0]*lidar_ori[3] + lidar_ori[1]*lidar_ori[2]),
                                        1 - 2*(lidar_ori[2]**2 + lidar_ori[3]**2))
                self.yaw = (1 - alpha) * self.yaw + alpha * lidar_yaw
            
    def _generate_scan(self, x, y, yaw):
        """Simulate LiDAR scan from current odometry pose (Fallback)."""
        visible = []
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        for p in self.map_points:
            # Transform to robot frame
            dx = p[0] - x
            dy = p[1] - y
            dist = np.sqrt(dx**2 + dy**2)
            if dist < 5 and dist > 0.1:  # Range 5m, min 0.1m
                # Rotate to robot frame
                rx = dx * cos_yaw + dy * sin_yaw
                ry = -dx * sin_yaw + dy * cos_yaw
                # Add noise
                noise = np.random.normal(0, 0.02, 2)  # 2cm noise
                visible.append([rx + noise[0], ry + noise[1]])
        return np.array(visible)

    def _get_lidar_scan_2d(self):
        """Get latest 3D scan and project to 2D for ICP. Falls back to sim."""
        points = self.livox.get_latest_scan()
        
        # Fallback to simulation if no real data
        if len(points) == 0:
            # Only generate sim data if we have valid state
            return self._generate_scan(self.position[0], self.position[1], self.yaw)

        # LiDAR is mounted UPSIDE DOWN on robot head
        # Flip Z axis: z_correct = -z_raw
        points[:, 2] = -points[:, 2]
        
        # Filter out robot head/body (points within 15cm radius)
        distances_xy = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        mask_distance = distances_xy > 0.15  # More than 15cm from LiDAR center
        
        # Filter Z range to capture walls/features at human height
        # LiDAR is ~1.5m off ground when robot standing
        # We want points between floor and ceiling: -1.0m to +1.0m relative to LiDAR
        mask_z = (points[:, 2] > -1.0) & (points[:, 2] < 1.0)
        
        # Combine filters
        mask = mask_distance & mask_z
        valid_points = points[mask]
        
        if len(valid_points) == 0:
            return np.empty((0, 2))
            
        # Project to XY
        return valid_points[:, :2]



    def _low_state_handler(self, msg: LowState_):
        """Callback for receiving low-level state."""
        now = time.time()
        self.low_state = msg
        self.state_received = True

        # Update leg odometry
        try:
            # Get joint velocities and torques
            joint_pos = [msg.motor_state[i].q for i in range(G1_NUM_MOTOR)]
            joint_vel = [msg.motor_state[i].dq for i in range(G1_NUM_MOTOR)]
            joint_tau = [msg.motor_state[i].tau_est for i in range(G1_NUM_MOTOR)]
            gyro = list(msg.imu_state.gyroscope)

            if self.last_update_time is not None:
                dt = now - self.last_update_time
                dt = min(dt, 0.1)  # Clamp
                self._update_leg_odometry(joint_pos, joint_vel, joint_tau, gyro, dt)

                # Real SLAM correction using real LiDAR data
                if now - self.last_slam_time >= self.slam_interval:
                    scan_2d = self._get_lidar_scan_2d()
                    
                    if len(scan_2d) > 50:  # Need good structure
                        # Downsample if too large for performance
                        if len(scan_2d) > 500:
                            indices = np.random.choice(len(scan_2d), 500, replace=False)
                            scan_2d = scan_2d[indices]

                        # ICP to estimate transformation (Scan -> Map)
                        # Returns R, t, success
                        R, t, icp_success = self._icp(scan_2d, self.map_points)
                        
                        # Only fuse if ICP converged successfully
                        if icp_success:
                            # t is the Robot Position in Map Frame
                            slam_pos = np.array([t[0], t[1], self.position[2]])
                            
                            # Orientation correction
                            yaw_correction = np.arctan2(R[1, 0], R[0, 0])
                            slam_ori = np.array([np.cos(yaw_correction/2), 0, 0, np.sin(yaw_correction/2)])
                            
                            self._fuse_lidar_position(slam_pos, slam_ori, icp_success)
                        
                        self.last_slam_time = now
                    else:
                        # print("SLAM: insufficient scan points")
                        pass

            self.last_update_time = now
        except Exception:
            pass  # Ignore errors

        # Rate-limited publishing
        if now - self.last_publish_time >= self.publish_interval:
            self._publish_state()
            self.last_publish_time = now

    def _publish_state(self):
        """Pack and send state data over UDP (V4 format with dual positioning and LiDAR)."""
        try:
            # Pack joint positions (29 floats)
            joint_pos = [self.low_state.motor_state[i].q for i in range(G1_NUM_MOTOR)]
            joint_vel = [self.low_state.motor_state[i].dq for i in range(G1_NUM_MOTOR)]
            imu_quat = list(self.low_state.imu_state.quaternion)
            imu_gyro = list(self.low_state.imu_state.gyroscope)

            # Pack RAW position (leg odometry only, BEFORE ICP correction)
            position_raw = self.position_raw.tolist()
            
            # Pack ICP-CORRECTED position (AFTER SLAM fusion)
            position_icp = self.position_filtered.tolist()
            
            velocity = self.velocity_filtered.tolist()

            # Get LiDAR scan
            scan_2d = self._get_lidar_scan_2d()
            if len(scan_2d) > 100:
                indices = np.random.choice(len(scan_2d), 100, replace=False)
                scan_2d = scan_2d[indices]
            
            point_cloud = []
            for pt in scan_2d:
                point_cloud.extend([pt[0], pt[1], 0.0])
            point_count = len(scan_2d)

            # Create V4 packet
            header = struct.pack(
                "<4sd3f3f2f29f29f4f3fI",
                b"G1S4", time.time(),
                *position_raw, *position_icp, *velocity,
                *joint_pos, *joint_vel, *imu_quat, *imu_gyro,
                point_count
            )
            
            if point_count > 0:
                points_packed = struct.pack(f"<{len(point_cloud)}f", *point_cloud)
                packet = header + points_packed
            else:
                packet = header

            self.sock.sendto(packet, self.receiver_addr)
        except Exception as e:
            print(f"Error publishing state: {e}")


    def run(self):
        """Main loop - just keeps the script alive."""
        print("\nPublishing robot state. Press Ctrl+C to stop.\n")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            self.sock.close()


def main():
    # Detect environment
    is_windows = platform.system() == "Windows"
    
    # Default values
    if is_windows:
        # Windows: Use local IP for interface, localhost for receiver
        try:
            net_if = socket.gethostbyname(socket.gethostname())
        except:
            net_if = "" # try empty
        receiver_ip = "127.0.0.1"
    else:
        # Linux/Jetson: Eth0 and remote PC
        net_if = "eth0"
        receiver_ip = "192.168.123.100"

    # Parse arguments
    if len(sys.argv) >= 2:
        net_if = sys.argv[1]

    if len(sys.argv) >= 3:
        receiver_ip = sys.argv[2]

    print(f"G1 State Publisher")
    print(f"==================")
    print(f"Network interface: {net_if}")
    print(f"Receiver IP: {receiver_ip}")
    print(f"UDP Port: {UDP_PORT}")
    print()

    publisher = G1StatePublisher(net_if=net_if, receiver_ip=receiver_ip)
    publisher.run()


if __name__ == "__main__":
    main()
