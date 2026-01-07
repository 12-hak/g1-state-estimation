#pragma once

#include <Eigen/Dense>
#include <vector>
#include <array>

class G1Kinematics {
public:
    G1Kinematics();

    // Leg Index: 0 = Left, 1 = Right
    // q: 6 joint angles for the leg
    Eigen::Vector3d ForwardKinematics(int leg_index, const Eigen::VectorXd& q_leg);

    Eigen::MatrixXd ComputeJacobian(int leg_index, const Eigen::VectorXd& q_leg);
    
    // Estimate Velocity of foot relative to base
    Eigen::Vector3d ComputeFootVelocity(int leg_index, const Eigen::VectorXd& q_leg, const Eigen::VectorXd& dq_leg);

private:
    // Parameters (Meters)
    // Total leg length ~0.6m. 
    // Approximating as symmetric 0.3m + 0.3m based on standard design if URDF not available.
    // TODO: Load from config or exact URDF if available.
    double l_hip_pitch_x = 0.0; 
    double l_hip_roll_y = 0.064; // Approx hip offset (check URDF)
    double l_thigh = 0.3;       
    double l_calf = 0.3;     
    // double l_foot_z = 0.0;   // Foot is point contact or small sphere
    
    // Helper for transforms
    Eigen::Matrix4d get_transform(double q, double a, double d, double alpha); // DH if needed, or simple rot
};
