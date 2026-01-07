#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "UnitreeInterface.hpp"
#include "G1Kinematics.hpp"
#include <array>

struct RobotState {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d linear_acceleration;
    
    // Joints
    std::array<float, 35> q;
    std::array<float, 35> dq;
};

class StateEstimator {
public:
    StateEstimator();
    
    void update_imu(const std::array<float, 4>& quat, 
                    const std::array<float, 3>& gyro, 
                    const std::array<float, 3>& accel);
                    
    void update_joints(const std::array<float, 35>& q, 
                       const std::array<float, 35>& dq,
                       const std::array<float, 35>& tau);

    void update_vision_pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& ori);

    RobotState get_state() const;

private:
    RobotState state_;
    bool initialized_ = false;

    // Filter biases or covariance could go here
    Eigen::Vector3d gyro_bias_;
    Eigen::Vector3d accel_bias_;
    
    G1Kinematics kinematics_;

    // Kalman Filter State
    // State Vector x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]
    Eigen::VectorXd x_hat_; // Estimated State (6x1)
    Eigen::MatrixXd P_;     // Error Covariance (6x6)
    Eigen::MatrixXd Q_;     // Process Noise Covariance (6x6)
    Eigen::MatrixXd R_;     // Measurement Noise Covariance (6x6) -- Assuming simple diag R
    
    // Process Model
    // x_k+1 = A x_k + B u_k
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    
    // Measurement Model
    // z = C x
    Eigen::MatrixXd C_;
    
    // IMU Data Store for Prediction
    Eigen::Vector3d last_accel_;
    Eigen::Quaterniond last_quat_;
    bool imu_initialized_ = false;
};
