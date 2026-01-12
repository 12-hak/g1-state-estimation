#!/usr/bin/env python3
"""
G1 Multi-Strategy Recovery Viewer
Kinematic ghost playback of three different standup strategies:
1. NINJA_PUSHUP: Face-down (Prone) recovery.
2. SUPINE_CRUNCH: Back (Supine) recovery using momentum.
3. KIP_UP: High-speed explosive pop.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    model = mujoco.MjModel.from_xml_path("unitree_mujoco/unitree_robots/g1/scene.xml")
    data = mujoco.MjData(model)
    
    # --- STRATEGY 1: NINJA PUSHUP (Feet Anchor) ---
    ninja_pushup = {
        "PRONE": {
            'pelvis_z': 0.12, 'pelvis_pitch': np.pi/2, 'pelvis_x': 0.0,
            'left_hip_pitch_joint': 0.0, 'left_knee_joint': 0.0, 'left_ankle_pitch_joint': 0.0
        },
        "ARM_PUSH": {
            'pelvis_z': 0.35, 'pelvis_pitch': 0.8, 'pelvis_x': 0.0,
            'left_shoulder_pitch_joint': 0.5, 'left_elbow_joint': 1.5,
            'left_hip_pitch_joint': -0.4, 'left_knee_joint': 0.8, 'left_ankle_pitch_joint': -0.4 # Knees stay on ground
        },
        "LEG_TUCK": {
            'pelvis_z': 0.45, 'pelvis_pitch': 0.3, 'pelvis_x': 0.2, # Pull root forward over knees
            'left_hip_pitch_joint': -1.8, 'left_knee_joint': 2.4, 'left_ankle_pitch_joint': -0.8
        },
        "FEET_ON_GROUND": {
            'pelvis_z': 0.48, 'pelvis_pitch': 0.2, 'pelvis_x': 0.3, # Anchor feet, move root up
            'left_hip_pitch_joint': -1.5, 'left_knee_joint': 2.0, 'left_ankle_pitch_joint': -0.5,
            'waist_pitch_joint': 1.2
        },
        "STAND": {
            'pelvis_z': 0.75, 'pelvis_pitch': 0.0, 'pelvis_x': 0.3, # Feet stay at x=0.3
            'left_hip_pitch_joint': -0.2, 'left_knee_joint': 0.4, 'left_ankle_pitch_joint': -0.2
        }
    }

    # --- STRATEGY 2: SUPINE CRUNCH (Heel Anchor) ---
    supine_crunch = {
        "BACK": {
            'pelvis_z': 0.1, 'pelvis_pitch': -np.pi/2, 'pelvis_x': 0.0,
            'left_hip_pitch_joint': -0.4, 'left_knee_joint': 0.8, 'left_ankle_pitch_joint': -0.4
        },
        "KNEE_PULL": {
            'pelvis_z': 0.15, 'pelvis_pitch': -np.pi/2, 'pelvis_x': 0.0,
            'left_hip_pitch_joint': -1.5, 'left_knee_joint': 2.2, 'left_ankle_pitch_joint': -0.8
        },
        "SIT_WEIGHT": {
            'pelvis_z': 0.28, 'pelvis_pitch': -0.4, 'pelvis_x': 0.0,
            'waist_pitch_joint': 0.8, 'left_hip_pitch_joint': -1.2, 'left_knee_joint': 1.5
        },
        "ANCHOR_FEET": {
            'pelvis_z': 0.42, 'pelvis_pitch': 0.1, 'pelvis_x': 0.15, # Rock root forward
            'left_hip_pitch_joint': -1.8, 'left_knee_joint': 2.4, 'left_ankle_pitch_joint': -0.6
        },
        "STAND": {
            'pelvis_z': 0.75, 'pelvis_pitch': 0.0, 'pelvis_x': 0.15,
            'left_hip_pitch_joint': -0.2, 'left_knee_joint': 0.4, 'left_ankle_pitch_joint': -0.2
        }
    }

    # --- STRATEGY 3: EXPLOSIVE KIP UP (Airborne Jump) ---
    kip_up = {
        "RELOAD": {
            'pelvis_z': 0.18, 'pelvis_pitch': -np.pi/2, 'pelvis_x': 0.0,
            'left_hip_pitch_joint': -1.8, 'left_knee_joint': 2.6
        },
        "KICK_UP": {
            'pelvis_z': 0.65, 'pelvis_pitch': -0.3, 'pelvis_x': 0.2, # OFF GROUND
            'left_hip_pitch_joint': 0.2, 'left_knee_joint': 0.0
        },
        "LAND_DYNAMIC": {
            'pelvis_z': 0.45, 'pelvis_pitch': 0.2, 'pelvis_x': 0.4, # Feet strike ground
            'waist_pitch_joint': 1.5, 'left_hip_pitch_joint': -1.8, 'left_knee_joint': 2.5
        },
        "STAND": {
            'pelvis_z': 0.75, 'pelvis_pitch': 0.0, 'pelvis_x': 0.4,
            'left_hip_pitch_joint': -0.2, 'left_knee_joint': 0.4
        }
    }

    strategies = [
        {"name": "NINJA_PUSHUP (Face Down)", "data": ninja_pushup, "keys": ["PRONE", "ARM_PUSH", "LEG_TUCK", "FEET_ON_GROUND", "STAND"]},
        {"name": "SUPINE_CRUNCH (Back)", "data": supine_crunch, "keys": ["BACK", "KNEE_PULL", "SIT_WEIGHT", "ANCHOR_FEET", "STAND"]},
        {"name": "KIP_UP (Explosive Pop)", "data": kip_up, "keys": ["RELOAD", "KICK_UP", "LAND_DYNAMIC", "STAND"]}
    ]

    phase_duration = 1.0
    start_time = time.time()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 90
        viewer.cam.lookat[:] = [0.2, 0, 0.4]

        while viewer.is_running():
            now = time.time()
            elapsed = now - start_time
            
            # Reset all to neutral
            data.qpos[7:] = model.qpos0[7:]
            
            # Draw ghost for each strategy
            for strat in strategies:
                sd = strat["data"]
                keys = strat["keys"]
                
                # Each strat exists in its own 'time space' to play simultaneously
                t = elapsed % (len(keys) * phase_duration)
                idx = int(t // phase_duration)
                alpha = (t % phase_duration) / phase_duration
                s_alpha = 3 * alpha**2 - 2 * alpha**3 # Smooth
                
                p1 = sd[keys[idx]]
                p2 = sd[keys[(idx + 1) % len(keys)]]
                
                # We can't actually draw 'ghosts' in simple viewer easily without extra bodies,
                # so we will use one strategy and tele-port it between them 
                # OR we can just playback one after another. 
                # Let's do Sequential Playback for clarity.
            
            # --- SEQUENTIAL PLAYBACK ---
            curr_strat_idx = int((elapsed // 5) % len(strategies))
            strat = strategies[curr_strat_idx]
            sd = strat["data"]
            keys = strat["keys"]
            
            t = elapsed % phase_duration
            idx = int((elapsed % (len(keys) * phase_duration)) // phase_duration)
            alpha = (elapsed % phase_duration) / phase_duration
            s_alpha = 3 * alpha**2 - 2 * alpha**3
            
            p1 = sd[keys[idx]]
            p2 = sd[keys[(idx + 1) % len(keys)]]
            
            # Apply Root
            x1, z1, pt1 = p1.get('pelvis_x', 0), p1['pelvis_z'], p1['pelvis_pitch']
            x2, z2, pt2 = p2.get('pelvis_x', 0), p2['pelvis_z'], p2['pelvis_pitch']
            data.qpos[0] = x1 + (x2 - x1) * s_alpha
            data.qpos[2] = z1 + (z2 - z1) * s_alpha
            
            pitch = pt1 + (pt2 - pt1) * s_alpha
            data.qpos[3] = np.cos(pitch/2)
            data.qpos[4] = 0
            data.qpos[5] = np.sin(pitch/2)
            data.qpos[6] = 0
            
            # Apply Joints
            all_j = set(p1.keys()) | set(p2.keys())
            for j in all_j:
                if 'pelvis' in j: continue
                try:
                    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                    q_addr = model.jnt_qposadr[jid]
                    v1 = p1.get(j, model.qpos0[q_addr]); v2 = p2.get(j, model.qpos0[q_addr])
                    data.qpos[q_addr] = v1 + (v2 - v1) * s_alpha
                    # Mirror to other side if joint name doesn't specify
                    opp = j.replace('left', 'right') if 'left' in j else j.replace('right', 'left')
                    ojid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, opp)
                    data.qpos[model.jnt_qposadr[ojid]] = v1 + (v2 - v1) * s_alpha
                except: pass

            mujoco.mj_forward(model, data)
            viewer.sync()
            
            if int(elapsed * 10) % 20 == 0:
                print(f"STRATEGY: {strat['name']} | PHASE: {keys[idx]:15s}", end="\r")
            
            time.sleep(0.01)

if __name__ == "__main__":
    main()
