#!/usr/bin/env python3
"""
B2 Flip Simulation Demo (Proxy for A2)
This demonstrates how to simulate robot flips in MuJoCo.
The same approach will work for A2 once the model is available.

The B2 is a quadruped similar to A2 and can be used to prototype
flip simulation concepts.
"""

import time
import sys
import numpy as np
from pathlib import Path

# Add unitree_sdk2py to path
sys.path.append(str(Path(__file__).parent / "unitree_sdk2" / "python"))

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

class FlipController:
    """
    Controller for executing flip maneuvers in simulation
    
    This demonstrates the control sequence needed for flips.
    For A2, you would use the sport mode API instead of low-level control.
    """
    
    def __init__(self, robot_type="b2"):
        self.robot_type = robot_type
        self.dt = 0.002  # Control timestep
        self.crc = CRC()
        
        # Initialize communication
        ChannelFactoryInitialize(1, "lo")
        
        # Create publisher
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()
        
        # Initialize command
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self.cmd.head[0] = 0xFE
        self.cmd.head[1] = 0xEF
        self.cmd.level_flag = 0xFF
        self.cmd.gpio = 0
        
        # Initialize motors
        for i in range(20):
            self.cmd.motor_cmd[i].mode = 0x01  # PMSM mode
            self.cmd.motor_cmd[i].q = 0.0
            self.cmd.motor_cmd[i].kp = 0.0
            self.cmd.motor_cmd[i].dq = 0.0
            self.cmd.motor_cmd[i].kd = 0.0
            self.cmd.motor_cmd[i].tau = 0.0
        
        print(f"[+] Initialized {robot_type.upper()} flip controller")
    
    def crouch_position(self):
        """Get crouching position for flip preparation"""
        # Deep crouch for explosive jump
        return np.array([
            0.0, 1.4, -2.8,  # Front left
            0.0, 1.4, -2.8,  # Front right
            0.0, 1.4, -2.8,  # Rear left
            0.0, 1.4, -2.8,  # Rear right
        ], dtype=float)
    
    def jump_position(self):
        """Get extended position for jump"""
        return np.array([
            0.0, 0.5, -1.0,  # Front left
            0.0, 0.5, -1.0,  # Front right
            0.0, 0.5, -1.0,  # Rear left
            0.0, 0.5, -1.0,  # Rear right
        ], dtype=float)
    
    def tuck_position(self):
        """Get tucked position for rotation"""
        return np.array([
            0.0, 2.0, -2.6,  # Front left
            0.0, 2.0, -2.6,  # Front right
            0.0, 2.0, -2.6,  # Rear left
            0.0, 2.0, -2.6,  # Rear right
        ], dtype=float)
    
    def land_position(self):
        """Get landing position"""
        return np.array([
            0.0, 0.9, -1.8,  # Front left
            0.0, 0.9, -1.8,  # Front right
            0.0, 0.9, -1.8,  # Rear left
            0.0, 0.9, -1.8,  # Rear right
        ], dtype=float)
    
    def interpolate_position(self, start_pos, end_pos, phase):
        """Smoothly interpolate between positions"""
        return phase * end_pos + (1 - phase) * start_pos
    
    def set_joint_positions(self, positions, kp=80.0, kd=5.0):
        """Set target joint positions"""
        for i in range(12):
            self.cmd.motor_cmd[i].q = positions[i]
            self.cmd.motor_cmd[i].kp = kp
            self.cmd.motor_cmd[i].kd = kd
            self.cmd.motor_cmd[i].dq = 0.0
            self.cmd.motor_cmd[i].tau = 0.0
    
    def send_command(self):
        """Send command to robot"""
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)
    
    def execute_front_flip(self):
        """
        Execute a front flip maneuver
        
        Sequence:
        1. Crouch (0.5s)
        2. Explosive jump (0.2s)
        3. Tuck and rotate (0.4s)
        4. Extend for landing (0.2s)
        5. Absorb impact (0.3s)
        """
        print("\n[*] Executing FRONT FLIP sequence...")
        
        start_time = time.perf_counter()
        running_time = 0.0
        
        # Get reference positions
        crouch = self.crouch_position()
        jump = self.jump_position()
        tuck = self.tuck_position()
        land = self.land_position()
        
        while running_time < 1.6:  # Total flip duration
            step_start = time.perf_counter()
            running_time = time.perf_counter() - start_time
            
            if running_time < 0.5:
                # Phase 1: Crouch
                phase = np.tanh(running_time / 0.3)
                pos = self.interpolate_position(jump, crouch, phase)
                self.set_joint_positions(pos, kp=60.0, kd=4.0)
                print(f"  Phase 1: Crouching... ({running_time:.2f}s)")
                
            elif running_time < 0.7:
                # Phase 2: Explosive jump
                phase = (running_time - 0.5) / 0.2
                pos = self.interpolate_position(crouch, jump, phase)
                self.set_joint_positions(pos, kp=120.0, kd=6.0)
                print(f"  Phase 2: Jumping! ({running_time:.2f}s)")
                
            elif running_time < 1.1:
                # Phase 3: Tuck and rotate
                phase = (running_time - 0.7) / 0.4
                pos = self.interpolate_position(jump, tuck, phase)
                self.set_joint_positions(pos, kp=100.0, kd=5.0)
                print(f"  Phase 3: Rotating... ({running_time:.2f}s)")
                
            elif running_time < 1.3:
                # Phase 4: Extend for landing
                phase = (running_time - 1.1) / 0.2
                pos = self.interpolate_position(tuck, land, phase)
                self.set_joint_positions(pos, kp=80.0, kd=5.0)
                print(f"  Phase 4: Extending for landing... ({running_time:.2f}s)")
                
            else:
                # Phase 5: Absorb impact
                phase = (running_time - 1.3) / 0.3
                pos = land
                self.set_joint_positions(pos, kp=60.0, kd=8.0)
                print(f"  Phase 5: Landing... ({running_time:.2f}s)")
            
            self.send_command()
            
            # Maintain control frequency
            time_until_next_step = self.dt - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        
        print("[+] Front flip complete!")
    
    def execute_back_flip(self):
        """Execute a back flip maneuver (similar to front flip but reversed)"""
        print("\n[*] Executing BACK FLIP sequence...")
        print("    (Implementation similar to front flip)")
        # Implementation would be similar but with different rotation direction
        self.execute_front_flip()  # Placeholder

def main():
    print("="*60)
    print("B2 Flip Simulation Demo (Proxy for A2)")
    print("="*60)
    print()
    print("This demonstrates flip simulation in MuJoCo.")
    print("The same approach will work for A2 when the model is available.")
    print()
    print("IMPORTANT: Run this with MuJoCo simulation active!")
    print("  1. In another terminal, start MuJoCo simulation:")
    print("     cd unitree_mujoco/simulate_python")
    print("     # Edit config.py: ROBOT = 'b2'")
    print("     python unitree_mujoco.py")
    print("  2. Then run this script")
    print()
    
    input("Press Enter when MuJoCo simulation is running...")
    
    # Create controller
    controller = FlipController(robot_type="b2")
    
    # Wait for initialization
    time.sleep(1.0)
    
    # Execute flip
    try:
        controller.execute_front_flip()
        
        print("\n" + "="*60)
        print("NOTES FOR A2 IMPLEMENTATION")
        print("="*60)
        print("For A2, you would use the Sport Mode API instead:")
        print()
        print("  from unitree_sdk2py.a2.sport import SportClient")
        print("  client = SportClient()")
        print("  client.Init()")
        print()
        print("  # Execute front flip")
        print("  client.FrontFlip()  # If API exists")
        print()
        print("  # Or trigger FSM state")
        print("  client.SwitchMode(ID_FRONT_FLIP)  # Mode 10")
        print()
        print("The flip would be handled by the robot's built-in controller,")
        print("making it much simpler than low-level joint control!")
        
    except KeyboardInterrupt:
        print("\n[!] Interrupted by user")
    except Exception as e:
        print(f"\n[-] Error: {e}")
        print("    Make sure MuJoCo simulation is running!")

if __name__ == "__main__":
    main()
