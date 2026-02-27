#include "g1_slam/ekf_fusion.hpp"
#include <cmath>
#include <algorithm>

namespace g1_slam {

EKFFusion::EKFFusion() {
    state_.setZero();
    P_ = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;

    Q_ = Eigen::Matrix<double, 6, 6>::Zero();
    Q_(0,0) = 0.001;  // x
    Q_(1,1) = 0.001;  // y
    Q_(2,2) = 0.005;  // z (less observable)
    Q_(3,3) = 0.001;  // roll
    Q_(4,4) = 0.001;  // pitch
    Q_(5,5) = 0.0005; // yaw
}

void EKFFusion::setPose(const Eigen::Matrix4d& pose) {
    state_.head<3>() = pose.block<3,1>(0,3);
    state_.tail<3>() = rotationMatrixToEuler(pose.block<3,3>(0,0));
    P_ *= 0.5;
}

void EKFFusion::reset() {
    state_.setZero();
    P_ = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
    initialized_ = false;
}

Eigen::Vector3d EKFFusion::rotationMatrixToEuler(const Eigen::Matrix3d& R) {
    double roll  = std::atan2(R(2,1), R(2,2));
    double pitch = std::asin(std::clamp(-R(2,0), -1.0, 1.0));
    double yaw   = std::atan2(R(1,0), R(0,0));
    return {roll, pitch, yaw};
}

Eigen::Matrix3d EKFFusion::eulerToRotationMatrix(const Eigen::Vector3d& euler) {
    return (Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

void EKFFusion::predict(const Eigen::Vector3d& linear_vel,
                         const Eigen::Vector3d& angular_vel,
                         double dt) {
    if (!initialized_) return;

    Eigen::Matrix3d R = eulerToRotationMatrix(state_.tail<3>());
    Eigen::Vector3d world_vel = R * linear_vel;

    // State prediction
    state_(0) += world_vel.x() * dt;
    state_(1) += world_vel.y() * dt;
    state_(2) += world_vel.z() * dt;
    state_(3) += angular_vel.x() * dt;
    state_(4) += angular_vel.y() * dt;
    state_(5) += angular_vel.z() * dt;

    // Normalize angles
    for (int i = 3; i < 6; ++i) {
        while (state_(i) > M_PI) state_(i) -= 2.0 * M_PI;
        while (state_(i) < -M_PI) state_(i) += 2.0 * M_PI;
    }

    // Jacobian of state transition (identity + derivatives)
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    // Position depends on orientation (simplified)
    F(0, 5) = -world_vel.y() * dt;
    F(1, 5) =  world_vel.x() * dt;

    // Covariance prediction
    P_ = F * P_ * F.transpose() + Q_ * dt;
}

void EKFFusion::updateLidar(const Eigen::Matrix4d& lidar_pose,
                             const Eigen::Matrix<double, 6, 6>& covariance) {
    Eigen::Matrix<double, 6, 1> z;
    z.head<3>() = lidar_pose.block<3,1>(0,3);
    z.tail<3>() = rotationMatrixToEuler(lidar_pose.block<3,3>(0,0));

    if (!initialized_) {
        state_ = z;
        P_ = covariance;
        initialized_ = true;
        return;
    }

    // Innovation
    Eigen::Matrix<double, 6, 1> y = z - state_;
    for (int i = 3; i < 6; ++i) {
        while (y(i) > M_PI) y(i) -= 2.0 * M_PI;
        while (y(i) < -M_PI) y(i) += 2.0 * M_PI;
    }

    // H = Identity (direct observation)
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> R = covariance;

    // Kalman gain
    Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, 6, 6> K = P_ * H.transpose() * S.inverse();

    // State update
    state_ += K * y;
    // Joseph form for numerical stability
    auto IKH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();

    // Normalize angles
    for (int i = 3; i < 6; ++i) {
        while (state_(i) > M_PI) state_(i) -= 2.0 * M_PI;
        while (state_(i) < -M_PI) state_(i) += 2.0 * M_PI;
    }
}

void EKFFusion::updateLegOdom(const Eigen::Vector3d& position,
                               const Eigen::Quaterniond& orientation,
                               const Eigen::Matrix<double, 6, 6>& covariance) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3,3>(0,0) = orientation.toRotationMatrix();
    pose.block<3,1>(0,3) = position;
    updateLidar(pose, covariance);
}

Eigen::Matrix4d EKFFusion::getPose() const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = eulerToRotationMatrix(state_.tail<3>());
    T.block<3,1>(0,3) = state_.head<3>();
    return T;
}

Eigen::Vector3d EKFFusion::getPosition() const {
    return state_.head<3>();
}

Eigen::Quaterniond EKFFusion::getOrientation() const {
    return Eigen::Quaterniond(eulerToRotationMatrix(state_.tail<3>()));
}

}  // namespace g1_slam
