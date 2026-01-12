#!/usr/bin/env python3
"""
G1 Trajectory Exporter
Samples the kinematic stand-up sequence at 50Hz and saves it to a CSV.
Use this CSV as a 'Reference Motion' for RL training.
"""

import mujoco
import numpy as np
import pandas as pd

def main():
    model = mujoco.MjModel.from_xml_path("unitree_mujoco/unitree_robots/g1/scene.xml")
    data = mujoco.MjData(model)
    
    # --- SAME PHASES AS VIEWER ---
    phases = {
        "SUPINE": {
            'pelvis_z': 0.08, 'pelvis_pitch': -np.pi/2,
            'left_hip_pitch_joint': -0.2, 'left_knee_joint': 0.4, 'left_ankle_pitch_joint': -0.2,
            'right_hip_pitch_joint': -0.2, 'right_knee_joint': 0.4, 'right_ankle_pitch_joint': -0.2,
            'waist_pitch_joint': 0.0, 'left_shoulder_pitch_joint': 0.0, 'right_shoulder_pitch_joint': 0.0
        },
        "KNEE_PULL": {
            'pelvis_z': 0.1, 'pelvis_pitch': -np.pi/2,
            'left_hip_pitch_joint': -1.2, 'left_knee_joint': 2.0, 'left_ankle_pitch_joint': -0.8,
            'right_hip_pitch_joint': -1.2, 'right_knee_joint': 2.0, 'right_ankle_pitch_joint': -0.8,
            'waist_pitch_joint': 0.0, 'left_shoulder_pitch_joint': 1.0, 'right_shoulder_pitch_joint': 1.0
        },
        "MOMENTUM_SIT": {
            'pelvis_z': 0.22, 'pelvis_pitch': -np.pi/4,
            'waist_pitch_joint': 1.2,
            'left_hip_pitch_joint': -1.0, 'left_knee_joint': 1.5,
            'right_hip_pitch_joint': -1.0, 'right_knee_joint': 1.5,
            'left_shoulder_pitch_joint': 1.8, 'right_shoulder_pitch_joint': 1.8,
            'left_elbow_joint': 0.5, 'right_elbow_joint': 0.5
        },
        "DEEP_TUCK": {
            'pelvis_z': 0.28, 'pelvis_pitch': 0.0,
            'waist_pitch_joint': 1.0,
            'left_hip_pitch_joint': -2.2, 'left_knee_joint': 2.8, 'left_ankle_pitch_joint': -1.2,
            'right_hip_pitch_joint': -2.2, 'right_knee_joint': 2.8, 'right_ankle_pitch_joint': -1.2,
            'left_shoulder_pitch_joint': 0.0, 'right_shoulder_pitch_joint': 0.0
        },
        "NOSE_OVER_TOES": {
            'pelvis_z': 0.35, 'pelvis_pitch': 0.2,
            'waist_pitch_joint': 1.4, 
            'left_hip_pitch_joint': -1.9, 'left_knee_joint': 2.4, 'left_ankle_pitch_joint': -0.6,
            'right_hip_pitch_joint': -1.9, 'right_knee_joint': 2.4, 'right_ankle_pitch_joint': -0.6,
            'left_shoulder_pitch_joint': 1.0, 'right_shoulder_pitch_joint': 1.0
        },
        "POWER_STAND": {
            'pelvis_z': 0.75, 'pelvis_pitch': 0.0,
            'waist_pitch_joint': 0.0,
            'left_hip_pitch_joint': -0.4, 'left_knee_joint': 0.8, 'left_ankle_pitch_joint': -0.4,
            'right_hip_pitch_joint': -0.4, 'right_knee_joint': 0.8, 'right_ankle_pitch_joint': -0.4
        }
    }

    phase_keys = ["SUPINE", "KNEE_PULL", "MOMENTUM_SIT", "DEEP_TUCK", "NOSE_OVER_TOES", "POWER_STAND"]
    phase_duration = 0.8 # Real-world speed for export
    sampling_freq = 50.0 # 50 Hz
    dt = 1.0 / sampling_freq
    
    total_time = len(phase_keys) * phase_duration
    num_steps = int(total_time / dt)
    
    trajectory_data = []

    print(f"Exporting {num_steps} samples to CSV...")

    for i in range(num_steps):
        t = i * dt
        idx = int(t // phase_duration)
        alpha = (t % phase_duration) / phase_duration
        s_alpha = 3 * alpha**2 - 2 * alpha**3
        
        p1 = phases[phase_keys[idx % len(phase_keys)]]
        p2 = phases[phase_keys[(idx + 1) % len(phase_keys)]]
        
        # Build state row
        row = {'time': t, 'phase': phase_keys[idx % len(phase_keys)]}
        
        # 1. Pelvis
        row['pelvis_z'] = p1['pelvis_z'] + (p2['pelvis_z'] - p1['pelvis_z']) * s_alpha
        row['pelvis_pitch'] = p1['pelvis_pitch'] + (p2['pelvis_pitch'] - p1['pelvis_pitch']) * s_alpha
        
        # 2. All Joints
        for jid in range(model.njnt):
            jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if jname == 'freejoint' or not jname: continue
            
            q_addr = model.jnt_qposadr[jid]
            v1 = p1.get(jname, model.qpos0[q_addr])
            v2 = p2.get(jname, model.qpos0[q_addr])
            row[jname] = v1 + (v2 - v1) * s_alpha
            
        trajectory_data.append(row)

    # Save to CSV
    df = pd.DataFrame(trajectory_data)
    output_file = "d:/python/state_e/g1_standup_ref.csv"
    df.to_csv(output_file, index=False)
    
    print(f"SUCCESS: Reference trajectory saved to '{output_file}'")
    print(f"Columns exported: {df.columns.tolist()[:10]} ... and more")

if __name__ == "__main__":
    main()
