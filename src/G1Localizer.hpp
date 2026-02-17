#pragma once

#include "ScanMatcher.hpp"
#include "LivoxInterface.hpp"
#include "UDPPublisher.hpp"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <atomic>
#include <thread>

namespace g1_localization {

struct LocalizationState {
    Eigen::Vector3f position;           // x, y, z
    Eigen::Vector3f velocity;           // vx, vy, vz
    Eigen::Quaternionf orientation;     // IMU orientation
    Eigen::Vector3f angular_velocity;   // gyro
    
    Eigen::Vector3f position_raw;       // Before ICP correction
    bool icp_valid;
    float icp_error;
    
    uint64_t timestamp_us;
};

class G1Localizer {
public:
    G1Localizer(const std::string& network_interface = "eth0",
                const std::string& receiver_ip = "192.168.123.100",
                uint16_t receiver_port = 9870);
    ~G1Localizer();
    
    void start();
    void stop();
    
    LocalizationState getState() const;
    
    // Visualization Accessors
    std::vector<LidarPoint> getGlobalMap() const;
    std::vector<Eigen::Vector2f> getLatestScan() const;

private:
    // DDS callbacks
    void lowStateCallback(const void* message);
    
    // Processing threads
    void localizationLoop();
    void publishLoop();
    
    // Leg odometry
    void updateLegOdometry(const std::vector<float>& joint_pos,
                          const std::vector<float>& joint_vel,
                          const std::vector<float>& joint_tau,
                          const Eigen::Vector3f& gyro,
                          float dt);
    
    bool detectSquat(const std::vector<float>& joint_vel) const;
    
    // LiDAR processing
    std::vector<Eigen::Vector2f> getLidarScan2D();
    std::vector<Eigen::Vector2f> generateMap();
    
    // Fusion
    void fuseLidarPosition(const Eigen::Vector2f& lidar_pos,
                          float lidar_yaw,
                          bool icp_success);
    
    // State
    LocalizationState state_;
    mutable std::mutex state_mutex_;
    
    // DDS
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> low_state_sub_;
    std::unique_ptr<unitree::robot::ChannelPublisher<LocalizationState>> state_pub_;
    
    // Components
    std::unique_ptr<ScanMatcher> matcher_;
    std::unique_ptr<LivoxInterface> livox_;
    std::unique_ptr<UDPPublisher> udp_publisher_;
    
    // Latest joint data for UDP
    std::vector<float> latest_joint_pos_;
    std::vector<float> latest_joint_vel_;
    std::vector<Eigen::Vector2f> latest_scan_2d_;
    
    // Map
    std::vector<LidarPoint> global_map_;
    
    // Threads
    std::atomic<bool> running_;
    std::thread localization_thread_;
    std::thread publish_thread_;
    
    // Timing
    std::chrono::steady_clock::time_point last_update_time_;
    std::chrono::steady_clock::time_point last_slam_time_;
    
    // Parameters
    Eigen::Vector3f lidar_offset_;
    float step_scale_;
    
    // SLAM Correction (Drift fix)
    Eigen::Vector3f slam_correction_;
    float slam_yaw_correction_;
    float position_alpha_;
    float velocity_alpha_;
    float slam_interval_;
    
    // Status
    std::atomic<bool> is_stationary_;
};

} // namespace g1_localization
