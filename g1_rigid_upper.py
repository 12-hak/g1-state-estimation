#!/usr/bin/env python3
"""
G1 Balance - Rigid Upper Body Strategy
Upper body (waist, torso, arms) locked rigid.
Only legs move for balance.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def quat_to_euler(quat):
    w, x, y, z = quat
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1, 1))
    return roll, pitch

def main():
    model = mujoco.MjModel.from_xml_path("unitree_mujoco/unitree_robots/g1/scene.xml")
    data = mujoco.MjData(model)
    
    print("=" * 70)
    print("G1 Balance - RIGID UPPER BODY")
    print("=" * 70)
    print("Strategy:")
    print("  - Waist/torso: EXTREMELY STIFF (like welded)")
    print("  - Arms: LOCKED in position")
    print("  - Legs: Active balance control")
    print("=" * 70)
    
    # Standing pose
    standing_pose = {
        # Legs - wide, stable stance
        'left_hip_pitch_joint': -0.5,
        'left_hip_roll_joint': 0.12,
        'left_hip_yaw_joint': 0.0,
        'left_knee_joint': 1.0,
        'left_ankle_pitch_joint': -0.5,
        'left_ankle_roll_joint': 0.0,
        
        'right_hip_pitch_joint': -0.5,
        'right_hip_roll_joint': -0.12,
        'right_hip_yaw_joint': 0.0,
        'right_knee_joint': 1.0,
        'right_ankle_pitch_joint': -0.5,
        'right_ankle_roll_joint': 0.0,
        
        # Waist - LOCKED STRAIGHT
        'waist_yaw_joint': 0.0,
        'waist_roll_joint': 0.0,
        'waist_pitch_joint': 0.0,
        
        # Arms - LOCKED wide for stability
        'left_shoulder_pitch_joint': 0.0,
        'left_shoulder_roll_joint': 0.4,  # Arms out
        'left_shoulder_yaw_joint': 0.0,
        'left_elbow_joint': 0.3,
        'left_wrist_roll_joint': 0.0,
        'left_wrist_pitch_joint': 0.0,
        
        'right_shoulder_pitch_joint': 0.0,
        'right_shoulder_roll_joint': -0.4,
        'right_shoulder_yaw_joint': 0.0,
        'right_elbow_joint': 0.3,
        'right_wrist_roll_joint': 0.0,
        'right_wrist_pitch_joint': 0.0,
    }
    
    # Set initial pose
    for joint_name, target in standing_pose.items():
        try:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            qpos_addr = model.jnt_qposadr[joint_id]
            data.qpos[qpos_addr] = target
        except:
            pass
    
    mujoco.mj_forward(model, data)
    
    # EXTREME STIFFNESS for upper body
    kp_leg = 2500.0
    kd_leg = 250.0
    kp_waist = 5000.0  # EXTREMELY STIFF
    kd_waist = 500.0
    kp_arm = 3000.0    # LOCKED
    kd_arm = 300.0
    
    # Balance gains
    balance_kp = 350.0
    balance_kd = 70.0
    
    print(f"Leg stiffness: KP={kp_leg}")
    print(f"Waist stiffness: KP={kp_waist} (RIGID)")
    print(f"Arm stiffness: KP={kp_arm} (LOCKED)")
    print(f"Balance gains: KP={balance_kp}")
    print("=" * 70)
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        last_print = time.time()
        
        while viewer.is_running():
            step_start = time.time()
            
            # Get IMU
            imu_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'imu')
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, data.site_xmat[imu_site_id].flatten())
            roll, pitch = quat_to_euler(quat)
            
            pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'pelvis')
            ang_vel = data.cvel[pelvis_id, 3:6]
            
            # Balance corrections (ONLY ankles and hips)
            ankle_pitch_corr = balance_kp * (-pitch) - balance_kd * ang_vel[1]
            ankle_roll_corr = balance_kp * (-roll) - balance_kd * ang_vel[0]
            hip_pitch_corr = 0.4 * ankle_pitch_corr
            hip_roll_corr = 0.4 * ankle_roll_corr
            
            # Gravity compensation
            qfrc_bias = np.zeros(model.nv)
            mujoco.mj_rne(model, data, 0, qfrc_bias)
            
            # Apply control
            for joint_name, target_pos in standing_pose.items():
                try:
                    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                    qpos_addr = model.jnt_qposadr[joint_id]
                    qvel_addr = model.jnt_dofadr[joint_id]
                    
                    current_pos = data.qpos[qpos_addr]
                    current_vel = data.qvel[qvel_addr]
                    
                    # Select gains - CRITICAL: upper body must be RIGID
                    if 'waist' in joint_name:
                        kp, kd = kp_waist, kd_waist
                    elif 'shoulder' in joint_name or 'elbow' in joint_name or 'wrist' in joint_name:
                        kp, kd = kp_arm, kd_arm
                    else:  # Legs
                        kp, kd = kp_leg, kd_leg
                    
                    # PD + gravity
                    tau = kp * (target_pos - current_pos) - kd * current_vel + qfrc_bias[qvel_addr]
                    
                    # Add balance ONLY to legs
                    if 'ankle_pitch' in joint_name:
                        tau += ankle_pitch_corr
                    elif 'ankle_roll' in joint_name:
                        tau += ankle_roll_corr
                    elif 'hip_pitch' in joint_name:
                        tau -= hip_pitch_corr
                    elif 'hip_roll' in joint_name:
                        tau -= hip_roll_corr
                    
                    # Apply
                    for i in range(model.nu):
                        if model.actuator_trnid[i, 0] == joint_id:
                            tau_min, tau_max = model.actuator_ctrlrange[i]
                            data.ctrl[i] = np.clip(tau, tau_min, tau_max)
                            break
                except:
                    pass
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # Status
            if time.time() - last_print > 2.0:
                com_pos = data.subtree_com[0]
                print(f"Roll: {np.degrees(roll):6.2f}°  Pitch: {np.degrees(pitch):6.2f}°  "
                      f"Height: {com_pos[2]:.3f}m")
                last_print = time.time()
            
            time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))

if __name__ == "__main__":
    main()
