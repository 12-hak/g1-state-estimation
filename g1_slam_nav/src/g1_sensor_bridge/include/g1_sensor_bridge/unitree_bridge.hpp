#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <g1_msgs/msg/robot_state.hpp>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>

#include <Eigen/Dense>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>

namespace g1_sensor_bridge {

class UnitreeBridgeNode : public rclcpp::Node {
public:
    explicit UnitreeBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~UnitreeBridgeNode() override;

private:
    void initDDS();
    void lowStateCallback(const void* message);
    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void publishTimerCallback();

    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<g1_msgs::msg::RobotState>::SharedPtr state_pub_;

    // ROS2 subscriber for velocity commands
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;

    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Unitree SDK2 DDS
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> dds_sub_;

    // Cached state
    struct CachedState {
        std::vector<float> joint_pos;
        std::vector<float> joint_vel;
        std::vector<float> joint_tau;
        Eigen::Quaternionf imu_quat = Eigen::Quaternionf::Identity();
        Eigen::Vector3f imu_gyro = Eigen::Vector3f::Zero();
        Eigen::Vector3f imu_accel = Eigen::Vector3f::Zero();
        bool valid = false;
    };
    CachedState cached_;
    std::mutex state_mutex_;

    // Parameters
    std::string network_interface_;
    std::string robot_type_;  // "g1" or "go2"
    int num_joints_;
    std::vector<std::string> joint_names_;

    void loadJointNames();
};

}  // namespace g1_sensor_bridge
