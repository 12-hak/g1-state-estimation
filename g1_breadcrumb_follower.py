#!/usr/bin/env python3
import socket
import struct
import time
import sys
import threading
import numpy as np
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_, unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

class BreadcrumbFollower:
    def __init__(self, iface):
        self.iface = iface
        self.recorded_path = []
        self.is_recording = False
        self.is_playing = False
        
        self.current_pos = np.zeros(2)
        self.current_yaw = 0.0
        self.front_obstacle_dist = 10.0
        
        # Remote buttons
        self.l1_pressed = False
        self.l2_pressed = False
        self.up_pressed = False
        self.down_pressed = False
        
        # Data reception flags for diagnostics
        self.received_dds = False
        self.received_udp = False
        
        # State locks
        self.lock = threading.Lock()
        
        # SDK components
        ChannelFactoryInitialize(0, self.iface)
        
        # Remote subscribe (G1 uses rt/lf/lowstate for wireless_remote)
        self.lowstate_sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
        self.lowstate_sub.Init(self._low_state_callback, 10)
        
        # Fallback: Sport mode state subscribe (For default position)
        self.sport_state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sport_state_sub.Init(self._sport_state_callback, 10)
        
        # Loco client for commands
        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(5.0)
        self.loco_client.Init()
        
        # UDP listener for G1Localizer (G1S4 format)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("0.0.0.0", 9870))
        self.udp_sock.settimeout(0.5)
        
        # Threads
        threading.Thread(target=self._udp_listener_loop, daemon=True).start()
        threading.Thread(target=self._diagnostic_loop, daemon=True).start()

        print(f"Breadcrumb Follower V2 Initialized on {self.iface}")
        print("Listening for Remote on rt/lf/lowstate")
        print("Listening for Position on UDP:9870 (G1Localizer)")
        print("L1 + Up: Start/Stop Recording | L2 + Down: Playback")

    def _diagnostic_loop(self):
        while True:
            time.sleep(5)
            with self.lock:
                status = f"[STATUS] DDS: {'OK' if self.received_dds else 'MISSING'}, UDP: {'OK' if self.received_udp else 'MISSING'}"
                if self.is_recording:
                    status += f" | RECORDING: {len(self.recorded_path)} pts"
                print(status)

    def _udp_listener_loop(self):
        # magical G1S4 packet: magic(4), timestamp(8), pos_raw(12), pos_icp(12), velocity(8), ...
        # Total header is ~308 bytes, but we only need magic + bits
        while True:
            try:
                data, _ = self.udp_sock.recvfrom(2048)
                if len(data) >= 42 and data[0:4] == b"G1S4":
                    self.received_udp = True
                    # position_icp is at offset 4 + 8 + 12 = 24
                    # yaw is at offset 4 + 8 + 12 + 12 + 8 + 116 + 116 = 276 (approximately)
                    # Actually, easier to use imu_quat at offset 276
                    
                    # Unpack: magic(4s), ts(d), pos_raw(3f), pos_icp(3f)
                    pos_icp = struct.unpack('<fff', data[24:36])
                    
                    # imu_quat at offset 308 - 4 - 32 - 12 = 260
                    # Let's check struct size: 4 + 8 + 12 + 12 + 8 + 116 + 116 + 16 + 12 + 4 = 308
                    # imu_quat (4*4=16 bytes) is at 4+8+12+12+8+116+116 = 276
                    quat = struct.unpack('<ffff', data[276:292]) # w, x, y, z
                    
                    with self.lock:
                        self.current_pos = np.array([pos_icp[0], pos_icp[1]])
                        self.current_yaw = self._get_yaw(quat)
                        
                        if self.is_recording:
                            self._add_breadcrumb()
                            
            except socket.timeout:
                continue
            except Exception as e:
                print(f"UDP Error: {e}")

    def _low_state_callback(self, msg: LowState_):
        self.received_dds = True
        d1 = msg.wireless_remote[2]
        d2 = msg.wireless_remote[3]
        
        l1 = (d1 >> 1) & 1
        l2 = (d1 >> 5) & 1
        up = (d2 >> 4) & 1
        down = (d2 >> 6) & 1
        
        with self.lock:
            if l1 and up and not (self.l1_pressed and self.up_pressed):
                self._toggle_recording()
            
            if l2 and down and not (self.l2_pressed and self.down_pressed):
                self._start_playback()
            
            self.l1_pressed = bool(l1)
            self.l2_pressed = bool(l2)
            self.up_pressed = bool(up)
            self.down_pressed = bool(down)

    def _get_yaw(self, quat):
        w, x, y, z = quat
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def _sport_state_callback(self, msg: SportModeState_):
        with self.lock:
            # Always update obstacle distance for safety
            self.front_obstacle_dist = msg.range_obstacle[0]
            
            # Only use sport mode for position if UDP localizer is NOT active
            if self.received_udp:
                return
                
            self.current_pos = np.array([msg.position[0], msg.position[1]])
            self.current_yaw = self._get_yaw(msg.imu_state.quaternion)
            
            if self.is_recording:
                self._add_breadcrumb()

    def _add_breadcrumb(self):
        # Should be called with lock held
        if not self.recorded_path or np.linalg.norm(self.current_pos - self.recorded_path[-1]) > 0.3:
            self.recorded_path.append(self.current_pos.copy())
            print(f"[REC] Point {len(self.recorded_path)}: {self.current_pos}")

    def _toggle_recording(self):
        self.is_recording = not self.is_recording
        if self.is_recording:
            self.recorded_path = []
            self.is_playing = False
            print("\n>>> RECORDING STARTED")
        else:
            print(f"\n>>> RECORDING STOPPED. Saved {len(self.recorded_path)} points.")

    def _start_playback(self):
        if self.is_playing:
            self.is_playing = False
            print("\n>>> PLAYBACK STOPPED")
            return

        if not self.recorded_path:
            print("\n!!! NO TRAIL RECORDED")
            return
        
        self.is_recording = False
        self.is_playing = True
        print(f"\n>>> PLAYBACK STARTED. Path contains {len(self.recorded_path)} points.")
        threading.Thread(target=self._playback_loop, daemon=True).start()

    def _playback_loop(self):
        target_index = 0
        path = list(self.recorded_path)
        K_LINEAR = 0.5
        K_ANGULAR = 1.0
        MAX_VEL = 0.6
        MAX_YAW = 0.8
        
        while self.is_playing and target_index < len(path):
            target = path[target_index]
            with self.lock:
                pos = self.current_pos.copy()
                yaw = self.current_yaw
                obs = self.front_obstacle_dist
            
            if obs < 0.8:
                self.loco_client.Move(0, 0, 0)
                time.sleep(0.5)
                continue

            diff = target - pos
            dist = np.linalg.norm(diff)
            
            if dist < 0.4:
                print(f">>> ARRIVED at breadcrumb {target_index}")
                target_index += 1
                continue
                
            target_yaw = np.arctan2(diff[1], diff[0])
            yaw_err = target_yaw - yaw
            while yaw_err > np.pi: yaw_err -= 2*np.pi
            while yaw_err < -np.pi: yaw_err += 2*np.pi
            
            if abs(yaw_err) > 0.8:
                vx = 0.0
                vyaw = np.clip(K_ANGULAR * yaw_err, -MAX_YAW, MAX_YAW)
            else:
                vx = np.clip(K_LINEAR * dist, 0.2, MAX_VEL)
                vyaw = np.clip(K_ANGULAR * yaw_err, -MAX_YAW, MAX_YAW)
            
            self.loco_client.Move(vx, 0.0, vyaw)
            time.sleep(0.05)
        
        self.loco_client.Move(0, 0, 0)
        print("\n>>> MISSION COMPLETE")
        self.is_playing = False

    def run(self):
        while True:
            time.sleep(1)

    def run(self):
        while True:
            time.sleep(1)

if __name__ == "__main__":
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    app = BreadcrumbFollower(iface)
    app.run()
