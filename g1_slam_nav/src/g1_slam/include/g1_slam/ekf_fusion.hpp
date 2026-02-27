#pragma once

#include <Eigen/Dense>

namespace g1_slam {

// 6-DOF EKF state: [x, y, z, roll, pitch, yaw]
// Fuses LiDAR odometry from KISS-ICP with leg odometry from Unitree

class EKFFusion {
public:
    EKFFusion();

    void predict(const Eigen::Vector3d& linear_vel,
                 const Eigen::Vector3d& angular_vel,
                 double dt);

    void updateLidar(const Eigen::Matrix4d& lidar_pose,
                     const Eigen::Matrix<double, 6, 6>& covariance);

    void updateLegOdom(const Eigen::Vector3d& position,
                       const Eigen::Quaterniond& orientation,
                       const Eigen::Matrix<double, 6, 6>& covariance);

    Eigen::Matrix4d getPose() const;
    Eigen::Vector3d getPosition() const;
    Eigen::Quaterniond getOrientation() const;
    Eigen::Matrix<double, 6, 6> getCovariance() const { return P_; }

    void setPose(const Eigen::Matrix4d& pose);
    void reset();

private:
    static Eigen::Vector3d rotationMatrixToEuler(const Eigen::Matrix3d& R);
    static Eigen::Matrix3d eulerToRotationMatrix(const Eigen::Vector3d& euler);

    // State: [x, y, z, roll, pitch, yaw]
    Eigen::Matrix<double, 6, 1> state_;
    Eigen::Matrix<double, 6, 6> P_;  // Covariance

    // Process noise
    Eigen::Matrix<double, 6, 6> Q_;

    bool initialized_ = false;
};

}  // namespace g1_slam
