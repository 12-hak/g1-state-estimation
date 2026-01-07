#include "StateEstimator.hpp"
#include <iostream>
#include <cmath>

StateEstimator::StateEstimator() {
    state_.position.setZero();
    state_.orientation.setIdentity();
    state_.velocity.setZero();
    state_.angular_velocity.setZero();
    state_.linear_acceleration.setZero();
    
    gyro_bias_.setZero();
    accel_bias_.setZero();

    // Initialize KF
    // State: [px, py, pz, vx, vy, vz]
    x_hat_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;

    // Process Noise Q (TUNING REQUIRED)
    // High Pos uncertainty, Med Vel uncertainty
    Q_ = Eigen::MatrixXd::Identity(6, 6);
    Q_.block<3,3>(0,0) *= 0.001; // Pos integration drift
    Q_.block<3,3>(3,3) *= 0.01;  // Vel noise from accel

    // Measurement Noise R (TUNING REQUIRED)
    // Leg Odometry velocity noise
    R_ = Eigen::MatrixXd::Identity(6, 6) * 0.1; // Trust Odom reasonably well

    // Transition A (Discrete time model updated in loop, but structure constant-ish)
    A_ = Eigen::MatrixXd::Identity(6, 6);
    // B maps Accel -> State (0.5*dt^2 for pos, dt for vel) - updated in loop
    B_ = Eigen::MatrixXd::Zero(6, 3);
    
    // Measurement Matrix C
    // We measure Velocity directly from Leg Odom? 
    // Leg Odom gives us v_body. 
    // Measurement z = [vx, vy, vz] (partial state observed? No, we observe velocity).
    // Actually, usually we observe Velocity from legs and Position from?
    // MIT Cheetah Style: Observe Velocity (Legs) and Height (Kinematics) + Feet hold position.
    // Simplified LKF: Measure Velocity.
    // C maps State [p, v] -> Meas [v]
    // z = [vx, vy, vz] -> C = [0, I]
    C_ = Eigen::MatrixXd::Zero(3, 6);
    C_.block<3,3>(0,3) = Eigen::Matrix3d::Identity();

    std::cout << "[StateEstimator] Initialized with MIT-style LKF." << std::endl;
}

void StateEstimator::update_imu(const std::array<float, 4>& quat, 
                                const std::array<float, 3>& gyro, 
                                const std::array<float, 3>& accel) {
    state_.orientation = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
    state_.angular_velocity = Eigen::Vector3d(gyro[0], gyro[1], gyro[2]);
    
    // Rotate Accel to World Frame (Gravity removal needed!)
    // Assuming IMU includes Gravity? Usually yes (9.81 on Z when flat).
    // a_world = R * a_body - g
    
    Eigen::Vector3d a_body(accel[0], accel[1], accel[2]);
    Eigen::Vector3d g_world(0, 0, -9.81);
    
    Eigen::Vector3d a_world = state_.orientation * a_body + g_world;
    state_.linear_acceleration = a_world;

    // KF Prediction Step
    double dt = 0.002; // Assume 500Hz, should measure real dt
    
    // A: x = x + v*dt
    A_.setIdentity();
    A_.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
    
    // B: v = v + a*dt, p = p + 0.5*a*dt^2
    B_.setZero();
    B_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * (0.5 * dt * dt);
    B_.block<3,3>(3,0) = Eigen::Matrix3d::Identity() * dt;
    
    // Predict
    x_hat_ = A_ * x_hat_ + B_ * a_world;
    P_ = A_ * P_ * A_.transpose() + Q_;
    
    // Store for retrieval
    state_.position = x_hat_.head<3>();
    state_.velocity = x_hat_.tail<3>();
}

void StateEstimator::update_joints(const std::array<float, 35>& q, 
                                   const std::array<float, 35>& dq,
                                   const std::array<float, 35>& tau) {
    state_.q = q;
    state_.dq = dq;
    
    // ... [Extract Vectors Logic Same as Before] ...
    // Extract Leg Joint Data
    Eigen::VectorXd q_left(6), dq_left(6), tau_left(6);
    Eigen::VectorXd q_right(6), dq_right(6), tau_right(6);
    
    for(int i=0; i<6; ++i) {
        q_left(i) = q[i]; dq_left(i) = dq[i]; tau_left(i) = tau[i];
        q_right(i) = q[6+i]; dq_right(i) = dq[6+i]; tau_right(i) = tau[6+i]; // Check offset
    }
    
    // Compute Foot Velocities
    Eigen::Vector3d v_foot_l = kinematics_.ComputeFootVelocity(0, q_left, dq_left);
    Eigen::Vector3d v_foot_r = kinematics_.ComputeFootVelocity(1, q_right, dq_right);
    
    // Contact Estimation
    double tau_knee_l = std::abs(tau_left(3));
    double tau_knee_r = std::abs(tau_right(3));
    double contact_thresh = 5.0; // Nm
    double prob_l = 1.0 / (1.0 + std::exp(-(tau_knee_l - contact_thresh)));
    double prob_r = 1.0 / (1.0 + std::exp(-(tau_knee_r - contact_thresh)));
    
    // Rotate Leg Velocity to World Frame for Measurement (using Orientation)
    // v_body_est_local = -v_foot
    // v_body_est_world = R * (-v_foot)
    Eigen::Vector3d v_l_world = state_.orientation * (-v_foot_l);
    Eigen::Vector3d v_r_world = state_.orientation * (-v_foot_r);
    
    // Measurement Update (Fuse Probabilistically)
    // Simpler: Weighted Average Measurement, then KF Update?
    // Or Independent Updates?
    // Let's do Weighted Avg Measurement z.
    
    double w_l = prob_l / (prob_l + prob_r + 1e-3);
    double w_r = prob_r / (prob_l + prob_r + 1e-3);
    
    Eigen::Vector3d z_vel = v_l_world * w_l + v_r_world * w_r;
    
    // MEASUREMENT UPDATE (Standard KF)
    // z = Hx + v
    // y = z - Hx
    // S = H P H' + R
    // K = P H' S^-1
    // x = x + K y
    // P = (I - KH) P
    
    Eigen::VectorXd z(3);
    z << z_vel(0), z_vel(1), z_vel(2);
    
    Eigen::VectorXd y = z - C_ * x_hat_;
    Eigen::MatrixXd S = C_ * P_ * C_.transpose() + R_; // R is constant 6x6? No R is 3x3 for Velocity meas.
    // Fix R definition to 3x3
    Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * 0.1; // Velocity noise
    // Dynamically Inflate R if NO CONTACT?
    if (prob_l < 0.1 && prob_r < 0.1) {
        R_vel *= 1000.0; // Trust prediction (IMU) when in air
    }

    Eigen::MatrixXd K = P_ * C_.transpose() * (C_ * P_ * C_.transpose() + R_vel).inverse();
    
    x_hat_ = x_hat_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(6,6) - K * C_) * P_;
    
    state_.position = x_hat_.head<3>();
    state_.velocity = x_hat_.tail<3>();
}
    
    // std::cout << "Est V: " << state_.velocity.transpose() << std::endl;
}

void StateEstimator::update_vision_pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& ori) {
    // Fuse vision pose
    // Simple reset for now (or blend)
    state_.position = pos;
    // state_.orientation = state_.orientation.slerp(0.5, ori); // Example naive fusion
}

RobotState StateEstimator::get_state() const {
    return state_;
}
