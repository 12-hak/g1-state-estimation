#!/usr/bin/env python3
"""
Unitree A2 Front Flip Simulation Demo
This script performs a front flip maneuver on the A2 proxy model in MuJoCo.

Usage:
1. Start the MuJoCo simulator:
   python unitree_mujoco/simulate_python/unitree_mujoco.py
2. Run this flip demo:
   python demo_a2_flip_simulation.py
"""

import time
import sys
import numpy as np
from pathlib import Path

# Fix SDK path
SDK_PATH = Path("unitree_sdk2_python")
if SDK_PATH.exists():
    sys.path.append(str(SDK_PATH))

try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC
except ImportError:
    print("[-] unitree_sdk2py not found. Please install it first:")
    print("    git clone https://github.com/unitreerobotics/unitree_sdk2_python.git")
    print("    cd unitree_sdk2_python && pip install -e .")
    sys.exit(1)

class A2FlipController:
    def __init__(self):
        self.dt = 0.002
        self.crc = CRC()
        
        # Initialize communication (Domain 1 for simulation)
        ChannelFactoryInitialize(1, "lo")
        
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()
        
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self.cmd.head[0] = 0xFE
        self.cmd.head[1] = 0xEF
        self.cmd.level_flag = 0xFF
        
        for i in range(20):
            self.cmd.motor_cmd[i].mode = 0x01
            self.cmd.motor_cmd[i].q = 0.0
            self.cmd.motor_cmd[i].kp = 0.0
            self.cmd.motor_cmd[i].dq = 0.0
            self.cmd.motor_cmd[i].kd = 0.0
            self.cmd.motor_cmd[i].tau = 0.0
            
        print("[+] A2 Flip Controller Initialized")

    def set_joints(self, q, kp=150.0, kd=10.0):
        # A2 has higher mass, so we use higher PD gains
        for i in range(12):
            self.cmd.motor_cmd[i].q = q[i]
            self.cmd.motor_cmd[i].kp = kp
            self.cmd.motor_cmd[i].kd = kd
            
    def send(self):
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def execute_flip(self):
        print("[*] Starting Front Flip Sequence...")
        
        # Phase timings
        crouch_t = 0.6
        jump_t = 0.15
        tuck_t = 0.45
        extend_t = 0.2
        land_t = 0.5
        
        # Positions (Abduction, Hip, Knee) x 4
        stand = np.array([0, 0.9, -1.8] * 4)
        crouch = np.array([0, 1.5, -2.7] * 4)
        jump = np.array([0, -0.2, -0.6] * 4) # Explosive extension
        tuck = np.array([0, 2.2, -2.5] * 4)
        land = np.array([0, 1.1, -2.2] * 4)
        
        start_time = time.time()
        
        while True:
            t = time.time() - start_time
            
            if t < crouch_t:
                # 1. Crouch
                alpha = t / crouch_t
                q = (1-alpha)*stand + alpha*crouch
                self.set_joints(q, kp=200, kd=15)
            elif t < crouch_t + jump_t:
                # 2. JUMP!
                alpha = (t - crouch_t) / jump_t
                q = (1-alpha)*crouch + alpha*jump
                self.set_joints(q, kp=500, kd=20) # Max power
            elif t < crouch_t + jump_t + tuck_t:
                # 3. TUCK
                alpha = (t - crouch_t - jump_t) / tuck_t
                q = (1-alpha)*jump + alpha*tuck
                self.set_joints(q, kp=150, kd=10)
            elif t < crouch_t + jump_t + tuck_t + extend_t:
                # 4. EXTEND
                alpha = (t - crouch_t - jump_t - tuck_t) / extend_t
                q = (1-alpha)*tuck + alpha*land
                self.set_joints(q, kp=100, kd=5)
            elif t < crouch_t + jump_t + tuck_t + extend_t + land_t:
                # 5. LAND
                self.set_joints(land, kp=250, kd=30) # High damping for landing
            else:
                break
                
            self.send()
            time.sleep(self.dt)
            
        print("[+] Flip Sequence Complete")

if __name__ == "__main__":
    ctrl = A2FlipController()
    input("Press Enter to initiate A2 Front Flip...")
    ctrl.execute_flip()
    # Return to stand
    time.sleep(0.5)
    ctrl.set_joints([0, 0.9, -1.8]*4, kp=100, kd=10)
    for _ in range(100):
        ctrl.send()
        time.sleep(0.01)
