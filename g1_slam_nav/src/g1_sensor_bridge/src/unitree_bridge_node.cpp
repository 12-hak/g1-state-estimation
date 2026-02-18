#include "g1_sensor_bridge/unitree_bridge.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

namespace g1_sensor_bridge {

// G1 joint name ordering per SDK documentation
static const std::vector<std::string> G1_JOINT_NAMES = {
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
    "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
    "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw"
};

static const std::vector<std::string> GO2_JOINT_NAMES = {
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf"
};

UnitreeBridgeNode::UnitreeBridgeNode(const rclcpp::NodeOptions& options)
    : Node("unitree_bridge_node", options) {

    declare_parameter("network_interface", "eth0");
    declare_parameter("robot_type", "g1");
    declare_parameter("publish_rate", 50.0);

    network_interface_ = get_parameter("network_interface").as_string();
    robot_type_ = get_parameter("robot_type").as_string();

    loadJointNames();

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/body", 50);
    state_pub_ = create_publisher<g1_msgs::msg::RobotState>("robot_state", 10);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel_stamped", 10,
        std::bind(&UnitreeBridgeNode::cmdVelCallback, this, std::placeholders::_1));

    double rate = get_parameter("publish_rate").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&UnitreeBridgeNode::publishTimerCallback, this));

    cached_.joint_pos.resize(num_joints_, 0.0f);
    cached_.joint_vel.resize(num_joints_, 0.0f);
    cached_.joint_tau.resize(num_joints_, 0.0f);

    initDDS();
    RCLCPP_INFO(get_logger(), "Unitree bridge initialized (type=%s, joints=%d, iface=%s)",
                robot_type_.c_str(), num_joints_, network_interface_.c_str());
}

UnitreeBridgeNode::~UnitreeBridgeNode() = default;

void UnitreeBridgeNode::loadJointNames() {
    if (robot_type_ == "go2") {
        joint_names_ = GO2_JOINT_NAMES;
        num_joints_ = 12;
    } else {
        joint_names_ = G1_JOINT_NAMES;
        num_joints_ = 29;
    }
}

void UnitreeBridgeNode::initDDS() {
    ChannelFactory::Instance()->Init(0, network_interface_);
    dds_sub_ = std::make_unique<ChannelSubscriber<LowState_>>("rt/lowstate");
    dds_sub_->InitChannel(
        [this](const void* msg) { this->lowStateCallback(msg); }, 10);
}

void UnitreeBridgeNode::lowStateCallback(const void* message) {
    const auto* msg = static_cast<const LowState_*>(message);
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (int i = 0; i < num_joints_; ++i) {
        cached_.joint_pos[i] = msg->motor_state()[i].q();
        cached_.joint_vel[i] = msg->motor_state()[i].dq();
        cached_.joint_tau[i] = msg->motor_state()[i].tau_est();
    }

    cached_.imu_quat = Eigen::Quaternionf(
        msg->imu_state().quaternion()[0],
        msg->imu_state().quaternion()[1],
        msg->imu_state().quaternion()[2],
        msg->imu_state().quaternion()[3]);

    cached_.imu_gyro = Eigen::Vector3f(
        msg->imu_state().gyroscope()[0],
        msg->imu_state().gyroscope()[1],
        msg->imu_state().gyroscope()[2]);

    cached_.imu_accel = Eigen::Vector3f(
        msg->imu_state().accelerometer()[0],
        msg->imu_state().accelerometer()[1],
        msg->imu_state().accelerometer()[2]);

    cached_.valid = true;
}

void UnitreeBridgeNode::publishTimerCallback() {
    CachedState snap;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!cached_.valid) return;
        snap = cached_;
    }

    auto stamp = now();

    // JointState
    auto js = sensor_msgs::msg::JointState();
    js.header.stamp = stamp;
    js.name = joint_names_;
    js.position.assign(snap.joint_pos.begin(), snap.joint_pos.end());
    js.velocity.assign(snap.joint_vel.begin(), snap.joint_vel.end());
    js.effort.assign(snap.joint_tau.begin(), snap.joint_tau.end());
    joint_pub_->publish(js);

    // IMU
    auto imu = sensor_msgs::msg::Imu();
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.orientation.w = snap.imu_quat.w();
    imu.orientation.x = snap.imu_quat.x();
    imu.orientation.y = snap.imu_quat.y();
    imu.orientation.z = snap.imu_quat.z();
    imu.angular_velocity.x = snap.imu_gyro.x();
    imu.angular_velocity.y = snap.imu_gyro.y();
    imu.angular_velocity.z = snap.imu_gyro.z();
    imu.linear_acceleration.x = snap.imu_accel.x();
    imu.linear_acceleration.y = snap.imu_accel.y();
    imu.linear_acceleration.z = snap.imu_accel.z();
    imu_pub_->publish(imu);

    // RobotState
    auto rs = g1_msgs::msg::RobotState();
    rs.header.stamp = stamp;
    rs.imu_orientation = imu.orientation;
    rs.imu_angular_velocity = imu.angular_velocity;
    rs.imu_linear_acceleration = imu.linear_acceleration;
    rs.joint_positions.assign(snap.joint_pos.begin(), snap.joint_pos.end());
    rs.joint_velocities.assign(snap.joint_vel.begin(), snap.joint_vel.end());
    rs.joint_torques.assign(snap.joint_tau.begin(), snap.joint_tau.end());
    rs.robot_type = (robot_type_ == "go2") ? 2 : 1;

    float total_vel = 0.0f;
    for (int i = 0; i < std::min(num_joints_, 12); ++i) {
        total_vel += std::abs(snap.joint_vel[i]);
    }
    float gyro_mag = snap.imu_gyro.norm();
    rs.is_stationary = (total_vel < 0.2f && gyro_mag < 0.02f);

    state_pub_->publish(rs);
}

void UnitreeBridgeNode::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr) {
    // Forward velocity commands to the robot via Unitree SDK2 DDS
    // Implementation depends on the specific robot control mode
    RCLCPP_WARN_ONCE(get_logger(), "cmd_vel forwarding not yet implemented for direct DDS control");
}

}  // namespace g1_sensor_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_sensor_bridge::UnitreeBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
