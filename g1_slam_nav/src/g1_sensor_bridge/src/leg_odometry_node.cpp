#include "g1_sensor_bridge/leg_odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>

namespace g1_sensor_bridge {

LegOdometryNode::LegOdometryNode(const rclcpp::NodeOptions& options)
    : Node("leg_odometry_node", options) {

    declare_parameter("robot_type", "g1");
    declare_parameter("step_scale", 0.6);
    declare_parameter("lateral_scale", 0.4);
    declare_parameter("velocity_deadband", 0.10);
    declare_parameter("stationary_joint_threshold", 0.2);
    declare_parameter("stationary_gyro_threshold", 0.02);
    declare_parameter("yaw_complementary_alpha", 0.05);
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("publish_tf", true);
    declare_parameter("publish_rate", 50.0);

    robot_type_ = get_parameter("robot_type").as_string();
    step_scale_ = get_parameter("step_scale").as_double();
    lateral_scale_ = get_parameter("lateral_scale").as_double();
    velocity_deadband_ = get_parameter("velocity_deadband").as_double();
    stationary_joint_threshold_ = get_parameter("stationary_joint_threshold").as_double();
    stationary_gyro_threshold_ = get_parameter("stationary_gyro_threshold").as_double();
    yaw_complementary_alpha_ = get_parameter("yaw_complementary_alpha").as_double();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&LegOdometryNode::jointStateCallback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/body", 50,
        std::bind(&LegOdometryNode::imuCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom/legs", 50);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    double rate = get_parameter("publish_rate").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&LegOdometryNode::publishOdometry, this));

    position_.z() = (robot_type_ == "g1") ? 0.793 : 0.3;

    RCLCPP_INFO(get_logger(), "Leg odometry initialized (type=%s, step_scale=%.2f)",
                robot_type_.c_str(), step_scale_);
}

void LegOdometryNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    latest_joint_pos_.assign(msg->position.begin(), msg->position.end());
    latest_joint_vel_.assign(msg->velocity.begin(), msg->velocity.end());
    latest_joint_tau_.assign(msg->effort.begin(), msg->effort.end());
}

void LegOdometryNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    latest_imu_quat_ = Eigen::Quaterniond(
        msg->orientation.w, msg->orientation.x,
        msg->orientation.y, msg->orientation.z);
    latest_imu_gyro_ = Eigen::Vector3d(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z);
}

bool LegOdometryNode::detectStationary(const std::vector<double>& joint_vel) const {
    double total = 0.0;
    int leg_joints = 12;
    for (int i = 0; i < std::min(leg_joints, static_cast<int>(joint_vel.size())); ++i) {
        total += std::abs(joint_vel[i]);
    }
    double gyro_mag = latest_imu_gyro_.norm();
    return (total < stationary_joint_threshold_ && gyro_mag < stationary_gyro_threshold_);
}

void LegOdometryNode::publishOdometry() {
    std::vector<double> j_pos, j_vel, j_tau;
    Eigen::Quaterniond imu_q;
    Eigen::Vector3d gyro;

    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        if (latest_joint_vel_.empty()) return;
        j_pos = latest_joint_pos_;
        j_vel = latest_joint_vel_;
        j_tau = latest_joint_tau_;
        imu_q = latest_imu_quat_;
        gyro = latest_imu_gyro_;
    }

    auto now_time = now();
    double dt = 0.02;
    if (!first_update_) {
        dt = std::min((now_time - last_update_time_).seconds(), 0.1);
    }
    first_update_ = false;
    last_update_time_ = now_time;

    // Yaw stabilization via complementary filter
    double imu_yaw = std::atan2(
        2.0 * (imu_q.w() * imu_q.z() + imu_q.x() * imu_q.y()),
        1.0 - 2.0 * (imu_q.y() * imu_q.y() + imu_q.z() * imu_q.z()));

    if (!fused_init_) {
        fused_yaw_ = imu_yaw;
        fused_init_ = true;
    } else {
        fused_yaw_ += gyro.z() * dt;
        // Normalize
        while (fused_yaw_ > M_PI) fused_yaw_ -= 2.0 * M_PI;
        while (fused_yaw_ < -M_PI) fused_yaw_ += 2.0 * M_PI;
        double diff = imu_yaw - fused_yaw_;
        if (diff > M_PI) diff -= 2.0 * M_PI;
        if (diff < -M_PI) diff += 2.0 * M_PI;
        fused_yaw_ += yaw_complementary_alpha_ * diff;
    }

    // Reconstruct stable quaternion: keep IMU roll/pitch, use fused yaw
    double roll = std::atan2(
        2.0 * (imu_q.w() * imu_q.x() + imu_q.y() * imu_q.z()),
        1.0 - 2.0 * (imu_q.x() * imu_q.x() + imu_q.y() * imu_q.y()));
    double pitch = std::asin(
        std::clamp(2.0 * (imu_q.w() * imu_q.y() - imu_q.z() * imu_q.x()), -1.0, 1.0));

    orientation_ = Eigen::AngleAxisd(fused_yaw_, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    angular_velocity_ = gyro;

    // Stationary detection
    if (detectStationary(j_vel)) {
        velocity_ = Eigen::Vector3d::Zero();
    } else {
        // Stance leg selection: pick leg with higher ground reaction torque
        double lt = 0.0, rt = 0.0;
        int half = (robot_type_ == "g1") ? 6 : 3;
        for (int i = 0; i < half && i < static_cast<int>(j_tau.size()); ++i) {
            lt += std::abs(j_tau[i]);
            rt += std::abs(j_tau[i + half]);
        }
        bool use_left = (lt > rt);
        int base_idx = use_left ? 0 : half;

        double fwd_vel = j_vel[base_idx] * step_scale_;
        int lat_idx = base_idx + 1;
        double lat_vel = (lat_idx < static_cast<int>(j_vel.size())) ? -j_vel[lat_idx] * lateral_scale_ : 0.0;

        if (std::abs(fwd_vel) < velocity_deadband_) fwd_vel = 0.0;
        if (std::abs(lat_vel) < velocity_deadband_) lat_vel = 0.0;

        double c = std::cos(fused_yaw_);
        double s = std::sin(fused_yaw_);
        velocity_.x() = fwd_vel * c - lat_vel * s;
        velocity_.y() = fwd_vel * s + lat_vel * c;
        velocity_.z() = 0.0;

        position_.x() += velocity_.x() * dt;
        position_.y() += velocity_.y() * dt;
    }

    // Publish odometry
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = now_time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = position_.x();
    odom.pose.pose.position.y = position_.y();
    odom.pose.pose.position.z = position_.z();
    odom.pose.pose.orientation.w = orientation_.w();
    odom.pose.pose.orientation.x = orientation_.x();
    odom.pose.pose.orientation.y = orientation_.y();
    odom.pose.pose.orientation.z = orientation_.z();

    // Covariance: higher uncertainty in z, roll, pitch (not observable from legs)
    odom.pose.covariance[0] = 0.05;   // x
    odom.pose.covariance[7] = 0.05;   // y
    odom.pose.covariance[14] = 1.0;   // z (poorly observed)
    odom.pose.covariance[21] = 0.1;   // roll
    odom.pose.covariance[28] = 0.1;   // pitch
    odom.pose.covariance[35] = 0.02;  // yaw

    odom.twist.twist.linear.x = velocity_.x();
    odom.twist.twist.linear.y = velocity_.y();
    odom.twist.twist.angular.z = angular_velocity_.z();

    odom.twist.covariance[0] = 0.1;
    odom.twist.covariance[7] = 0.1;
    odom.twist.covariance[35] = 0.05;

    odom_pub_->publish(odom);

    // Publish TF
    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = base_frame_;
        tf.transform.translation.x = position_.x();
        tf.transform.translation.y = position_.y();
        tf.transform.translation.z = position_.z();
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }
}

}  // namespace g1_sensor_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_sensor_bridge::LegOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
