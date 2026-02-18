#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <string>

namespace g1_sensor_bridge {

class LegOdometryNode : public rclcpp::Node {
public:
    explicit LegOdometryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void publishOdometry();
    bool detectStationary(const std::vector<double>& joint_vel) const;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();

    // Yaw stabilization (complementary filter)
    double fused_yaw_ = 0.0;
    bool fused_init_ = false;

    // Cached sensor data
    std::vector<double> latest_joint_pos_;
    std::vector<double> latest_joint_vel_;
    std::vector<double> latest_joint_tau_;
    Eigen::Quaterniond latest_imu_quat_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d latest_imu_gyro_ = Eigen::Vector3d::Zero();
    std::mutex sensor_mutex_;
    rclcpp::Time last_update_time_;
    bool first_update_ = true;

    // Parameters
    std::string robot_type_;
    double step_scale_;
    double lateral_scale_;
    double velocity_deadband_;
    double stationary_joint_threshold_;
    double stationary_gyro_threshold_;
    double yaw_complementary_alpha_;
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;
};

}  // namespace g1_sensor_bridge
