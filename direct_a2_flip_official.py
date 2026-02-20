import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import ctypes

# Unitree A2 Front Flip Simulation - OFFICIAL URDF PARAMETERS
MODEL_PATH = "unitree_mujoco/unitree_robots/a2_official/scene.xml"

# Windows Key Code for Spacebar
VK_SPACE = 0x20

def is_key_pressed(key_code):
    return ctypes.windll.user32.GetAsyncKeyState(key_code) & 0x8000

# States
STATE_SETTLE    = 0  
STATE_IDLE      = 1  
STATE_CROUCH    = 2 
STATE_PUNCH     = 3 
STATE_RECOVER   = 4

def simulate_flip():
    if not os.path.exists(MODEL_PATH):
        print(f"[-] Error: {MODEL_PATH} not found.")
        return

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    model.opt.timestep = 0.001 
    data = mujoco.MjData(model)

    # Motor Targets (Mapped: FL, FR, RL, RR)
    # Using more aggressive pose for heavier robot
    P_STAND  = [0, 0.9, -1.8]
    P_CROUCH = [0, 1.5, -2.6]
    P_TUCK   = [0, 2.2, -2.7] 
    
    # --- DOUBLE PUNCH STRATEGY ---
    TORQUE_REAR  = 280.0 # Base for the double burst
    TORQUE_FRONT = TORQUE_REAR * 0.25
    
    PULSE_1      = 0.10 # Initial lift
    PULSE_GAP    = 0.04 # Rapid separation
    PULSE_2      = 0.08 # Secondary snap for rotation
    
    RELOCK_DELAY = 0.05 
    
    state = STATE_SETTLE
    sim_start_time = time.time()
    phase_start_t = 0
    last_space_state = False

    print("==========================================")
    print("      A2 DOUBLE PUNCH FRONT FLIP          ")
    print("==========================================")
    print(f"[+] Pulse Sequence: {PULSE_1}s -> {PULSE_GAP}s -> {PULSE_2}s")
    print(f"[+] Peak Torque:    {TORQUE_REAR} Nm")
    print("==========================================")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        mujoco.mj_resetData(model, data)
        data.qpos[2] = 0.55
        data.qpos[7:19] = P_STAND * 4
        mujoco.mj_forward(model, data)
        
        while viewer.is_running():
            step_start = time.time()
            t_sim = time.time() - sim_start_time
            
            # Key Detection
            space_pressed = is_key_pressed(VK_SPACE)
            space_trigger = space_pressed and not last_space_state
            last_space_state = space_pressed

            # --- CONTROL MACHINE ---
            target_q = np.array(P_STAND * 4)
            Kp = np.array([1200.0] * 12) 
            Kd = np.array([60.0] * 12)
            manual_torque = np.zeros(12)

            if state == STATE_SETTLE:
                if t_sim > 2.0:
                    state = STATE_IDLE
                    print("[READY] --- SPACE TO CROUCH ---")
            
            elif state == STATE_IDLE:
                if space_trigger:
                    state = STATE_CROUCH
                    phase_start_t = t_sim
                    print("[ACTION] --- Starting Crouch Sequence ---")

            elif state == STATE_CROUCH:
                t_p = t_sim - phase_start_t
                alpha = min(1.0, t_p / 1.5)
                target_q = (1-alpha)*np.array(P_STAND*4) + alpha*np.array(P_CROUCH*4)
                Kp[:] = 1800.0
                if t_p > 2.0:
                    if space_trigger:
                        state = STATE_PUNCH
                        phase_start_t = t_sim
                        print(f"[!!!] DOUBLE PUNCH INITIATED")

            elif state == STATE_PUNCH:
                t_p = t_sim - phase_start_t
                
                # Leg Targets for Clearance
                P_STAND_FRONT = [0, 0.9, -1.8]
                P_EXTEND_BACK  = [0, 0.4, -0.8] # Even straighter for clearance
                
                # --- DOUBLE PUNCH LOGIC ---
                if t_p < PULSE_1:
                    # PUNCH 1: Initial Lift
                    Kp[:] = 0.0 
                    Kd[:] = 1.0 
                    manual_torque[1], manual_torque[2] = -TORQUE_FRONT, TORQUE_FRONT
                    manual_torque[4], manual_torque[5] = -TORQUE_FRONT, TORQUE_FRONT
                    manual_torque[7], manual_torque[8] = -TORQUE_REAR, TORQUE_REAR
                    manual_torque[10], manual_torque[11] = -TORQUE_REAR, TORQUE_REAR
                
                elif t_p < PULSE_1 + PULSE_GAP:
                    # GAP: Coasting to preserve momentum
                    Kp[:] = 0.0
                    Kd[:] = 1.0
                
                elif t_p < PULSE_1 + PULSE_GAP + PULSE_2:
                    # PUNCH 2: Rotational Snap
                    Kp[:] = 0.0
                    Kd[:] = 1.0
                    # Secondary torque burst (Rear only for rotation)
                    manual_torque[7], manual_torque[8] = -TORQUE_REAR * 1.2, TORQUE_REAR * 1.2
                    manual_torque[10], manual_torque[11] = -TORQUE_REAR * 1.2, TORQUE_REAR * 1.2
                
                elif t_p < PULSE_1 + PULSE_GAP + PULSE_2 + RELOCK_DELAY:
                    # Final clearance coast
                    target_q[0:6]  = P_STAND_FRONT * 2
                    target_q[6:12] = P_EXTEND_BACK * 2
                    Kp[:] = 150.0 
                    Kd[:] = 10.0
                
                else:
                    # AERIAL RELOCK
                    target_q[0:6]  = P_STAND_FRONT * 2
                    target_q[6:12] = P_EXTEND_BACK * 2
                    Kp[:] = 3500.0 # Snap hard into catch position
                    Kd[:] = 150.0
                    
                    if t_p > 0.9: 
                        state = STATE_RECOVER
                        phase_start_t = t_sim
                        print("[ACTION] --- CATCHING LANDING ---")

            elif state == STATE_RECOVER:
                t_p = t_sim - phase_start_t
                target_q[:] = P_STAND * 4
                Kp[:] = 2000.0
                Kd[:] = 400.0 # Extreme damping for landing stability
                
                if t_p > 1.5:
                    if space_trigger:
                        state = STATE_SETTLE
                        sim_start_time = time.time()
                        mujoco.mj_resetData(model, data)
                        data.qpos[2] = 0.55
                        data.qpos[7:19] = P_STAND * 4
                        print("[RESET]")

            # Final Control Mix
            q_err = target_q - data.qpos[7:19]
            v_err = 0 - data.qvel[6:18]
            data.ctrl[:] = Kp * q_err + v_err * Kd + manual_torque

            mujoco.mj_step(model, data)
            viewer.sync()

            dt = model.opt.timestep
            elapsed = time.time() - step_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

if __name__ == "__main__":
    simulate_flip()
