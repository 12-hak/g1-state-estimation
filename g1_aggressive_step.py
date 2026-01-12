#!/usr/bin/env python3
"""
G1 Active Balance - Rigid Upper + Aggressive Stepping
Combines rigid upper body with very aggressive stepping to maintain balance.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
from enum import Enum

class StepState(Enum):
    DOUBLE_SUPPORT = 1  # Both feet on ground
    STEP_FORWARD_LEFT = 2
    STEP_FORWARD_RIGHT = 3
    STEP_BACKWARD_LEFT = 4
    STEP_BACKWARD_RIGHT = 5

def quat_to_euler(quat):
    w, x, y, z = quat
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1, 1))
    return roll, pitch

class AggressiveSteppingController:
    def __init__(self):
        self.state = StepState.DOUBLE_SUPPORT
        self.step_timer = 0.0
        self.step_duration = 0.18  # Ultra-fast steps (was 0.25)
        self.last_leg = 'RIGHT'
        
        # INSANE thresholds - zero tolerance for tipping
        self.pitch_threshold = 0.012  # ~0.7 degrees
        self.roll_threshold = 0.025   # ~1.4 degrees
        
        # Track triggers
        self.trigger_pitch = 0.0
        self.trigger_pitch_vel = 0.0
        self.trigger_roll = 0.0
        self.trigger_roll_vel = 0.0
        
        # Base pose
        self.base_pose = {
            'left_hip_pitch_joint': -0.4,
            'left_hip_roll_joint': 0.15,
            'left_hip_yaw_joint': 0.0,
            'left_knee_joint': 0.8,
            'left_ankle_pitch_joint': -0.4,
            'left_ankle_roll_joint': 0.0,
            'right_hip_pitch_joint': -0.4,
            'right_hip_roll_joint': -0.15,
            'right_hip_yaw_joint': 0.0,
            'right_knee_joint': 0.8,
            'right_ankle_pitch_joint': -0.4,
            'right_ankle_roll_joint': 0.0,
            'waist_yaw_joint': 0.0,
            'waist_roll_joint': 0.0,
            'waist_pitch_joint': 0.0,  
            'left_shoulder_pitch_joint': 0.8,
            'left_shoulder_roll_joint': 1.4,
            'left_elbow_joint': 1.5,
            'right_shoulder_pitch_joint': 0.8,
            'right_shoulder_roll_joint': -1.4,
            'right_elbow_joint': 1.5,
        }
        
        # PEAK STIFFNESS
        self.kp_leg = 7000.0  
        self.kd_leg = 700.0
        self.kp_waist = 20000.0  # WELDED
        self.kd_waist = 2000.0
        self.kp_arm = 8000.0
        self.kd_arm = 800.0
        
        self.balance_kp = 1800.0 # Violent ankle correction (was 1500)
        self.balance_kd = 400.0  # More damping for the violent correction
    
    def should_step(self, pitch, roll, ang_vel):
        """Zero-tolerance stepping trigger"""
        if self.state != StepState.DOUBLE_SUPPORT:
            return None
        
        # Predicive score (looking 150ms into the future)
        p_score = pitch + 0.15 * ang_vel[1]
        r_score = roll + 0.12 * ang_vel[0]
        
        self.trigger_pitch = pitch
        self.trigger_pitch_vel = ang_vel[1]
        self.trigger_roll = roll
        self.trigger_roll_vel = ang_vel[0]
        
        if p_score > self.pitch_threshold:
            if self.last_leg == 'RIGHT':
                self.last_leg = 'LEFT'; return StepState.STEP_FORWARD_LEFT
            else:
                self.last_leg = 'RIGHT'; return StepState.STEP_FORWARD_RIGHT
        elif p_score < -self.pitch_threshold:
            if self.last_leg == 'RIGHT':
                self.last_leg = 'LEFT'; return StepState.STEP_BACKWARD_LEFT
            else:
                self.last_leg = 'RIGHT'; return StepState.STEP_BACKWARD_RIGHT
        
        if abs(r_score) > self.roll_threshold:
            if r_score > 0: return StepState.STEP_FORWARD_RIGHT
            else: return StepState.STEP_FORWARD_LEFT
        
        return None
    
    def get_target_pose(self):
        """Dynamic targets with ACTIVE TORSO COUNTER-LEANING"""
        pose = self.base_pose.copy()
        if self.state == StepState.DOUBLE_SUPPORT:
            return pose
        
        phase = min(self.step_timer / self.step_duration, 1.0)
        swing = np.sin(phase * np.pi)
        
        # Dynamic magnitude
        pitch_step_scale = 0.85 + 2.5 * abs(self.trigger_pitch) + 0.8 * abs(self.trigger_pitch_vel)
        pitch_step_scale = np.clip(pitch_step_scale, 0.6, 1.5)
        
        # ACTIVE TORSO COUNTER-LEAN (Lean away from fall to move COM)
        lean_mag = 0.25 * swing
        if self.state in [StepState.STEP_FORWARD_LEFT, StepState.STEP_FORWARD_RIGHT]:
            pose['waist_pitch_joint'] = -lean_mag # Lean back while stepping forward
        else:
            pose['waist_pitch_joint'] = lean_mag # Lean forward while stepping backward
            
        weight_shift_mag = 0.25 * swing
        
        if self.state == StepState.STEP_FORWARD_LEFT:
            pose['left_hip_pitch_joint'] = -0.4 - pitch_step_scale * phase
            pose['left_knee_joint'] = 0.8 + 1.4 * swing
            pose['right_hip_roll_joint'] = -0.15 - weight_shift_mag
            pose['left_hip_roll_joint'] = 0.15 - weight_shift_mag
            
        elif self.state == StepState.STEP_FORWARD_RIGHT:
            pose['right_hip_pitch_joint'] = -0.4 - pitch_step_scale * phase
            pose['right_knee_joint'] = 0.8 + 1.4 * swing
            pose['left_hip_roll_joint'] = 0.15 + weight_shift_mag
            pose['right_hip_roll_joint'] = -0.15 + weight_shift_mag
        
        elif self.state == StepState.STEP_BACKWARD_LEFT:
            pose['left_hip_pitch_joint'] = -0.4 + pitch_step_scale * phase
            pose['left_knee_joint'] = 0.8 + 1.4 * swing
            pose['right_hip_roll_joint'] = -0.15 - weight_shift_mag
            pose['left_hip_roll_joint'] = 0.15 - weight_shift_mag
        
        elif self.state == StepState.STEP_BACKWARD_RIGHT:
            pose['right_hip_pitch_joint'] = -0.4 + pitch_step_scale * phase
            pose['right_knee_joint'] = 0.8 + 1.4 * swing
            pose['left_hip_roll_joint'] = 0.15 + weight_shift_mag
            pose['right_hip_roll_joint'] = -0.15 + weight_shift_mag
        
        return pose
        
        return pose
    
    def update(self, dt, pitch, roll, ang_vel):
        """Update state machine"""
        if self.state != StepState.DOUBLE_SUPPORT:
            self.step_timer += dt
            if self.step_timer >= self.step_duration:
                print(f"  -> Returning to DOUBLE_SUPPORT")
                self.state = StepState.DOUBLE_SUPPORT
                self.step_timer = 0.0
        else:
            new_state = self.should_step(pitch, roll, ang_vel)
            if new_state:
                self.state = new_state
                self.step_timer = 0.0
                print(f"STEP TRIGGERED: {new_state.name} (pitch={np.degrees(pitch):.1f}°, roll={np.degrees(roll):.1f}°)")

def main():
    model = mujoco.MjModel.from_xml_path("unitree_mujoco/unitree_robots/g1/scene.xml")
    data = mujoco.MjData(model)
    
    controller = AggressiveSteppingController()
    
    print("=" * 70)
    print("G1 AGGRESSIVE STEPPING BALANCE")
    print("=" * 70)
    print(f"Step threshold: {np.degrees(controller.pitch_threshold):.1f}° (VERY AGGRESSIVE)")
    print(f"Waist lean: {np.degrees(controller.base_pose['waist_pitch_joint']):.1f}° (backward)")
    print(f"Upper body: KP={controller.kp_waist} (RIGID)")
    print()
    print("Robot will step EARLY and OFTEN to maintain balance!")
    print("=" * 70)
    
    # Set initial pose
    for joint_name, target in controller.base_pose.items():
        try:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            qpos_addr = model.jnt_qposadr[joint_id]
            data.qpos[qpos_addr] = target
        except:
            pass
    
    mujoco.mj_forward(model, data)
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        last_time = time.time()
        last_print = time.time()
        
        while viewer.is_running():
            current_time = time.time()
            dt = min(current_time - last_time, 0.1)  # Cap dt
            last_time = current_time
            
            # Get IMU
            imu_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'imu')
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, data.site_xmat[imu_site_id].flatten())
            roll, pitch = quat_to_euler(quat)
            
            pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'pelvis')
            ang_vel = data.cvel[pelvis_id, 3:6]
            lin_vel = data.cvel[pelvis_id, 0:3] # Linear velocity
            
            # Update stepping
            controller.update(dt, pitch, roll, ang_vel)
            
            # Get target pose
            target_pose = controller.get_target_pose()
            
            # Balance corrections
            ankle_pitch_bal = controller.balance_kp * (-pitch) - controller.balance_kd * ang_vel[1]
            ankle_roll_bal = controller.balance_kp * (-roll) - controller.balance_kd * ang_vel[0]
            
            # TORSO VELOCITY DAMPING (Virtual force against falling)
            torso_pitch_damp = -500.0 * lin_vel[0] # Against forward/back velocity
            torso_roll_damp = -500.0 * lin_vel[1]  # Against left/right velocity
            
            # Gravity comp
            qfrc_bias = np.zeros(model.nv)
            mujoco.mj_rne(model, data, 0, qfrc_bias)
            
            # Apply control
            for joint_name, target_pos in target_pose.items():
                try:
                    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                    qpos_addr = model.jnt_qposadr[joint_id]
                    qvel_addr = model.jnt_dofadr[joint_id]
                    
                    current_pos = data.qpos[qpos_addr]
                    current_vel = data.qvel[qvel_addr]
                    
                    # Select gains
                    if 'waist' in joint_name:
                        kp, kd = controller.kp_waist, controller.kd_waist
                    elif 'shoulder' in joint_name or 'elbow' in joint_name or 'wrist' in joint_name:
                        kp, kd = controller.kp_arm, controller.kd_arm
                    else:
                        kp, kd = controller.kp_leg, controller.kd_leg
                    
                    # ACTIVE ANKLE LEVELING
                    # Foot should stay parallel to floor: Ankle_Pitch = -(Hip_Pitch + Knee_Pitch + Torso_Pitch)
                    if 'ankle_pitch' in joint_name:
                        side = 'left' if 'left' in joint_name else 'right'
                        hip_pitch = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{side}_hip_pitch_joint')]]
                        knee_pitch = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{side}_knee_joint')]]
                        # Leveling target
                        target_pos = -(hip_pitch + knee_pitch + pitch)
                    
                    if 'ankle_roll' in joint_name:
                        side = 'left' if 'left' in joint_name else 'right'
                        hip_roll = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{side}_hip_roll_joint')]]
                        # Leveling target
                        target_pos = -(hip_roll + roll)

                    tau = kp * (target_pos - current_pos) - kd * current_vel + qfrc_bias[qvel_addr]
                    
                    # COORDINATED BALANCE (Ankle + Hip Strategy)
                    if 'ankle_pitch' in joint_name:
                        tau += ankle_pitch_bal + torso_pitch_damp
                    elif 'ankle_roll' in joint_name:
                        tau += ankle_roll_bal + torso_roll_damp
                    elif 'hip_pitch' in joint_name:
                        # Hips provide major authority to push pelvis back
                        # (Forward fall -> Flex hips -> Pull pelvis back)
                        tau += (torso_pitch_damp * 1.5 + ankle_pitch_bal * 0.8)
                    elif 'hip_roll' in joint_name:
                        # Roll hips to help roll balance
                        tau += (torso_roll_damp * 0.8 + ankle_roll_bal * 0.4)
                    
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
            if current_time - last_print > 1.0:
                print(f"State: {controller.state.name:20s} | Pitch: {np.degrees(pitch):6.2f}° | Roll: {np.degrees(roll):6.2f}°")
                last_print = current_time
            
            time.sleep(max(0, model.opt.timestep - (time.time() - current_time)))

if __name__ == "__main__":
    main()
