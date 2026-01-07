#include "G1Kinematics.hpp"
#include <iostream>

G1Kinematics::G1Kinematics() {
    // TODO: Load accurate params from URDF or config
}

Eigen::Vector3d G1Kinematics::ForwardKinematics(int leg_index, const Eigen::VectorXd& q) {
    // q order: HipPitch, HipRoll, HipYaw, Knee, AnklePitch, AnkleRoll
    // Frame: Base -> Hip -> ... -> Foot
    // Note: Axis orientations are critical. Assuming standard logical frames provided by Unitree often match:
    // Pitch: Y-axis
    // Roll: X-axis
    // Yaw: Z-axis
    
    // Simplification for the "Robust" estimator first pass: 
    // We treat the legs as simple chains. 
    // Real implementation needs exact offsets.
    
    double side = (leg_index == 0) ? 1.0 : -1.0; // Left vs Right
    
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    // Base to Hip Center
    // T.translate(Eigen::Vector3d(0, side * l_hip_roll_y, 0));
    
    // Hip Pitch (Y)
    Eigen::AngleAxisd rot_hip_pitch(q(0), Eigen::Vector3d::UnitY());
    T.block<3,3>(0,0) *= rot_hip_pitch.toRotationMatrix();
    
    // Hip Roll (X)
    Eigen::AngleAxisd rot_hip_roll(q(1), Eigen::Vector3d::UnitX());
    T.block<3,3>(0,0) *= rot_hip_roll.toRotationMatrix();

    // Hip Yaw (Z)
    Eigen::AngleAxisd rot_hip_yaw(q(2), Eigen::Vector3d::UnitZ());
    T.block<3,3>(0,0) *= rot_hip_yaw.toRotationMatrix();

    // Thigh Link
    // Assuming Thigh extends downwards or forwards? usually downwards for neutral q=0?
    // Let's assume neutral q means leg straight down for G1? Or "sitting"?
    // Unitree usually: 0 is "hanging" or "standing"? 
    // G1 standing: Knee is bent? 
    // Let's assume Link is along -Z in local frame after hip.
    T.block<3,1>(0,3) += T.block<3,3>(0,0) * Eigen::Vector3d(0, 0, -l_thigh); 
    // Wait, this applies translation *after* rotation accumulator? 
    // We strictly chain Transforms.
    
    // Refined FK Chain:
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    
    // 0: HipPitch (Y)
    R = R * Eigen::AngleAxisd(q(0), Eigen::Vector3d::UnitY());
    
    // 1: HipRoll (X)
    R = R * Eigen::AngleAxisd(q(1), Eigen::Vector3d::UnitX());
    
    // 2: HipYaw (Z)
    R = R * Eigen::AngleAxisd(q(2), Eigen::Vector3d::UnitZ());
    
    // Move Length Thigh
    // Knee (Y-axis rotation typically)
    // Offset to Knee?
    p += R * Eigen::Vector3d(0, 0, -l_thigh);
    
    // 3: Knee (Y)
    R = R * Eigen::AngleAxisd(q(3), Eigen::Vector3d::UnitY());
    
    // Move Length Calf
    p += R * Eigen::Vector3d(0, 0, -l_calf);
    
    // 4: Ankle Pitch (Y)
    R = R * Eigen::AngleAxisd(q(4), Eigen::Vector3d::UnitY());
    
    // 5: Ankle Roll (X)
    R = R * Eigen::AngleAxisd(q(5), Eigen::Vector3d::UnitX());
    
    // Foot center offset?
    // p += R * Eigen::Vector3d(0, 0, -0.05); // Foot height
    
    return p;
}

Eigen::MatrixXd G1Kinematics::ComputeJacobian(int leg_index, const Eigen::VectorXd& q) {
    Eigen::MatrixXd J(3, 6);
    double eps = 1e-5;
    
    Eigen::Vector3d p0 = ForwardKinematics(leg_index, q);
    
    for(int i=0; i<6; ++i) {
        Eigen::VectorXd q_plus = q;
        q_plus(i) += eps;
        
        Eigen::Vector3d p_plus = ForwardKinematics(leg_index, q_plus);
        
        J.col(i) = (p_plus - p0) / eps;
    }
    
    return J;
}

Eigen::Vector3d G1Kinematics::ComputeFootVelocity(int leg_index, const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
    Eigen::MatrixXd J = ComputeJacobian(leg_index, q);
    return J * dq;
}
