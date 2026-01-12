#!/usr/bin/env python3
"""
G1 Kinematic Trajectory Viewer (Pure Playback)
Bypasses physics to show exactly what the planned motion sequence looks like.
No gravity, no forces, no "shooting off" - just smooth joint and root playback.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    # Load the scene
    model = mujoco.MjModel.from_xml_path("unitree_mujoco/unitree_robots/g1/scene.xml")
    data = mujoco.MjData(model)
    
    # --- DEFINE KEY POSES (Joints + Pelvis State) ---
    # pelvis_z: Height of pelvis
    # pelvis_pitch: Forward tilt (approximate for visualization)
    phases = {
        "SUPINE": {
            'pelvis_z': 0.08, 'pelvis_pitch': -np.pi/2, # Flat on back
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
            'pelvis_z': 0.22, 'pelvis_pitch': -np.pi/4, # Pivot to sitting
            'waist_pitch_joint': 1.2,
            'left_hip_pitch_joint': -1.0, 'left_knee_joint': 1.5,
            'right_hip_pitch_joint': -1.0, 'right_knee_joint': 1.5,
            'left_shoulder_pitch_joint': 1.8, 'right_shoulder_pitch_joint': 1.8,
            'left_elbow_joint': 0.5, 'right_elbow_joint': 0.5
        },
        "DEEP_TUCK": {
            'pelvis_z': 0.28, 'pelvis_pitch': 0.0, # Torso vertical
            'waist_pitch_joint': 1.0,
            'left_hip_pitch_joint': -2.2, 'left_knee_joint': 2.8, 'left_ankle_pitch_joint': -1.2,
            'right_hip_pitch_joint': -2.2, 'right_knee_joint': 2.8, 'right_ankle_pitch_joint': -1.2,
            'left_shoulder_pitch_joint': 0.0, 'right_shoulder_pitch_joint': 0.0
        },
        "NOSE_OVER_TOES": {
            'pelvis_z': 0.35, 'pelvis_pitch': 0.2, # Lean way forward
            'waist_pitch_joint': 1.4, 
            'left_hip_pitch_joint': -1.9, 'left_knee_joint': 2.4, 'left_ankle_pitch_joint': -0.6,
            'right_hip_pitch_joint': -1.9, 'right_knee_joint': 2.4, 'right_ankle_pitch_joint': -0.6,
            'left_shoulder_pitch_joint': 1.0, 'right_shoulder_pitch_joint': 1.0
        },
        "POWER_STAND": {
            'pelvis_z': 0.75, 'pelvis_pitch': 0.0, # Full vertical
            'waist_pitch_joint': 0.0,
            'left_hip_pitch_joint': -0.4, 'left_knee_joint': 0.8, 'left_ankle_pitch_joint': -0.4,
            'right_hip_pitch_joint': -0.4, 'right_knee_joint': 0.8, 'right_ankle_pitch_joint': -0.4
        }
    }

    phase_keys = ["SUPINE", "KNEE_PULL", "MOMENTUM_SIT", "DEEP_TUCK", "NOSE_OVER_TOES", "POWER_STAND"]
    current_phase_idx = 0
    start_time = time.time()
    phase_duration = 1.2
    
    print("=" * 70)
    print("G1 PURE KINEMATIC PLAYBACK (No Physics Forces)")
    print("=" * 70)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 2.0
        viewer.cam.azimuth = 90
        
        while viewer.is_running():
            now = time.time()
            elapsed = (now - start_time)
            
            # --- CALCULATE INTERPOLATION ---
            phase_time = elapsed % (len(phase_keys) * phase_duration)
            idx = int(phase_time // phase_duration)
            alpha = (phase_time % phase_duration) / phase_duration
            
            # Smooth interpolation
            s_alpha = 3 * alpha**2 - 2 * alpha**3
            
            p1 = phases[phase_keys[idx]]
            p2 = phases[phase_keys[(idx + 1) % len(phase_keys)]]
            
            # --- UPDATE STATE ---
            # 1. Root Position (Pelvis)
            data.qpos[0] = 0 # X
            data.qpos[1] = 0 # Y
            z_start = p1['pelvis_z']
            z_end = p2['pelvis_z']
            data.qpos[2] = z_start + (z_end - z_start) * s_alpha
            
            # 2. Root Orientation (Pitch)
            pitch_start = p1['pelvis_pitch']
            pitch_end = p2['pelvis_pitch']
            current_pitch = pitch_start + (pitch_end - pitch_start) * s_alpha
            
            # Convert pitch to quat [w, 0, sin(p/2), 0]
            data.qpos[3] = np.cos(current_pitch/2)
            data.qpos[4] = 0
            data.qpos[5] = np.sin(current_pitch/2)
            data.qpos[6] = 0
            
            # 3. Joints
            # Reset all to neutral first
            data.qpos[7:] = model.qpos0[7:]
            
            # Blend active joints
            all_joints = set(p1.keys()) | set(p2.keys())
            for jname in all_joints:
                if jname.startswith('pelvis'): continue
                
                try:
                    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                    q_addr = model.jnt_qposadr[jid]
                    
                    v1 = p1.get(jname, model.qpos0[q_addr])
                    v2 = p2.get(jname, model.qpos0[q_addr])
                    data.qpos[q_addr] = v1 + (v2 - v1) * s_alpha
                except: pass

            # --- RENDER ---
            # Forward only - bypasses physics solver, just updates visual positions
            mujoco.mj_forward(model, data)
            viewer.sync()
            
            # Status
            if int(elapsed * 10) % 15 == 0:
                print(f"Viewing Phase: {phase_keys[idx]:15s}", end="\r")
            
            time.sleep(0.01)

if __name__ == "__main__":
    main()
