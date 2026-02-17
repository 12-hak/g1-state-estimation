#!/usr/bin/env python3
import time
import sys
import struct
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
        
        # State locks
        self.lock = threading.Lock()
        
        # SDK components
        ChannelFactoryInitialize(0, self.iface)
        
        # Remote subscribe (G1 uses rt/lf/lowstate for wireless_remote)
        self.lowstate_sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
        self.lowstate_sub.Init(self._low_state_callback, 10)
        
        # Sport mode state subscribe (For position and obstacles)
        self.sport_state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sport_state_sub.Init(self._sport_state_callback, 10)
        
        # Loco client for commands
        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(5.0)
        self.loco_client.Init()
        
        print(f"Breadcrumb Follower Initialized on {self.iface}")
        print("L1 + Up: Start/Stop Recording")
        print("L2 + Down: Playback recorded trail")

    def _low_state_callback(self, msg: LowState_):
        # Extract buttons from wireless_remote (40 bytes)
        # data1 = msg.wireless_remote[2], data2 = msg.wireless_remote[3]
        d1 = msg.wireless_remote[2]
        d2 = msg.wireless_remote[3]
        
        # Mapping from unitree_sdk2py example
        l1 = (d1 >> 1) & 1
        l2 = (d1 >> 5) & 1
        up = (d2 >> 4) & 1
        down = (d2 >> 6) & 1
        
        with self.lock:
            # Detect L1 + Up Toggle
            if l1 and up and not (self.l1_pressed and self.up_pressed):
                self._toggle_recording()
            
            # Detect L2 + Down Toggle
            if l2 and down and not (self.l2_pressed and self.down_pressed):
                self._start_playback()
            
            self.l1_pressed = bool(l1)
            self.l2_pressed = bool(l2)
            self.up_pressed = bool(up)
            self.down_pressed = bool(down)

    def _get_yaw(self, quat):
        # quat: [w, x, y, z]
        w, x, y, z = quat
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def _sport_state_callback(self, msg: SportModeState_):
        with self.lock:
            self.current_pos = np.array([msg.position[0], msg.position[1]])
            self.current_yaw = self._get_yaw(msg.imu_state.quaternion)
            self.front_obstacle_dist = msg.range_obstacle[0]
            
            if self.is_recording:
                if not self.recorded_path or np.linalg.norm(self.current_pos - self.recorded_path[-1]) > 0.3:
                    self.recorded_path.append(self.current_pos.copy())
                    print(f"[REC] Point {len(self.recorded_path)}: {self.current_pos}")

    def _toggle_recording(self):
        self.is_recording = not self.is_recording
        if self.is_recording:
            self.recorded_path = []
            self.is_playing = False # Stop playback if recording starts
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
        print("\n>>> PLAYBACK STARTED. Moving to start of trail...")
        
        # Start playback thread
        threading.Thread(target=self._playback_loop, daemon=True).start()

    def _playback_loop(self):
        target_index = 0
        path = list(self.recorded_path)
        
        # Gains
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
            
            # 1. OBSTACLE SAFETY
            if obs < 0.8: # G1 is fast, keep a safe buffer
                print(f"!!! STOP: Obstacle at {obs:.2f}m")
                self.loco_client.Move(0, 0, 0)
                time.sleep(0.5)
                continue

            # 2. NAVIGATION MATH
            # World-frame error
            diff = target - pos
            dist = np.linalg.norm(diff)
            
            if dist < 0.4: # Arrival radius
                print(f">>> ARRIVED at breadcrumb {target_index}")
                target_index += 1
                continue
                
            # Desired heading
            target_yaw = np.arctan2(diff[1], diff[0])
            yaw_err = target_yaw - yaw
            
            # Wrap yaw error to [-pi, pi]
            while yaw_err > np.pi: yaw_err -= 2*np.pi
            while yaw_err < -np.pi: yaw_err += 2*np.pi
            
            # Simple Controller
            # We want to primarily turn towards the goal, then move
            if abs(yaw_err) > 0.8: # Big turn needed
                vx = 0.0
                vyaw = np.clip(K_ANGULAR * yaw_err, -MAX_YAW, MAX_YAW)
            else: # Move while turning
                vx = np.clip(K_LINEAR * dist, 0.2, MAX_VEL)
                vyaw = np.clip(K_ANGULAR * yaw_err, -MAX_YAW, MAX_YAW)
            
            # G1 Move API: Move(vx, vy, vyaw)
            # Body frame: vx forward, vy lateral (left positive)
            self.loco_client.Move(vx, 0.0, vyaw)
            
            time.sleep(0.05)
        
        # Stop at end
        self.loco_client.Move(0, 0, 0)
        print("\n>>> MISSION COMPLETE")
        self.is_playing = False

    def run(self):
        while True:
            time.sleep(1)

if __name__ == "__main__":
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    app = BreadcrumbFollower(iface)
    app.run()
