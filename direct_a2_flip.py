import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import ctypes

# Unitree A2 Front Flip Simulation
# Strategy: Balanced Launch + Landing Extension (Aesthetics Optimized)
MODEL_PATH = "unitree_mujoco/unitree_robots/a2/scene.xml"

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
    P_STAND  = [0, 0.9, -1.8]
    P_CROUCH = [0, 1.4, -2.5]
    P_TUCK   = [0, 2.0, -2.5] 
    
    # --- BALANCED POWER SETTINGS (SLIGHTLY REDUCED) ---
    # Previous: 180 Nm -> New: 155 Nm
    # Previous: 40 Nm -> New: 35.0 Nm
    TORQUE_REAR  = 155.0 
    TORQUE_FRONT = 35.0  
    PULSE_DURATION = 0.12 
    
    state = STATE_SETTLE
    sim_start_time = time.time()
    phase_start_t = 0
    last_space_state = False

    print("==========================================")
    print("      A2 BALANCED FRONT FLIP          ")
    print("==========================================")
    print(f"[+] Rear Torque:  {TORQUE_REAR} Nm")
    print(f"[+] Front Torque: {TORQUE_FRONT} Nm")
    print("[+] Status: READY FOR CALIBRATED LAUNCH")
    print("==========================================")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        mujoco.mj_resetData(model, data)
        data.qpos[2] = 0.40 
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
            Kp = np.array([800.0] * 12)
            Kd = np.array([50.0] * 12)
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
                Kp[:] = 1200.0
                if t_p > 2.0:
                    if space_trigger:
                        state = STATE_PUNCH
                        phase_start_t = t_sim
                        print(f"[!!!] BALANCED PUNCH: {TORQUE_REAR} Nm / {TORQUE_FRONT} Nm")

            elif state == STATE_PUNCH:
                t_p = t_sim - phase_start_t
                
                if t_p < PULSE_DURATION:
                    # --- UNLOCKED PUNCH ---
                    Kp[:] = 0.0 
                    Kd[:] = 1.0 
                    
                    # Front Assist (FL: 1,2 | FR: 4,5)
                    manual_torque[1] = -TORQUE_FRONT
                    manual_torque[2] = TORQUE_FRONT
                    manual_torque[4] = -TORQUE_FRONT
                    manual_torque[5] = TORQUE_FRONT
                    
                    # Rear Power (RL: 7,8 | RR: 10,11)
                    manual_torque[7] = -TORQUE_REAR
                    manual_torque[8] = TORQUE_REAR
                    manual_torque[10] = -TORQUE_REAR
                    manual_torque[11] = TORQUE_REAR
                else:
                    # --- AERIAL ROTATION & EXTENSION ---
                    # Briefly tuck for 0.4s then extend
                    if t_p < 0.5:
                        target_q[:] = P_TUCK * 4
                        Kp[:] = 2000.0
                    else:
                        state = STATE_RECOVER
                        phase_start_t = t_sim
                        print("[ACTION] --- CATCHING LANDING ---")

            elif state == STATE_RECOVER:
                t_p = t_sim - phase_start_t
                # Return to stand pose to take ground contact
                target_q[:] = P_STAND * 4
                Kp[:] = 1000.0
                Kd[:] = 250.0 # High damping to kill momentum
                
                if t_p > 1.5:
                    if space_trigger:
                        state = STATE_SETTLE
                        sim_start_time = time.time()
                        mujoco.mj_resetData(model, data)
                        data.qpos[2] = 0.40
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
