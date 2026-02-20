#pragma once

#include "ScanMatcher.hpp"
#include "PoseGraph.hpp"
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
#include <mutex>

namespace g1_localization {

struct LocalizationState {
    Eigen::Vector3f position;           // x, y, z (from pose graph when available)
    Eigen::Vector3f velocity;           // vx, vy, vz
    Eigen::Quaternionf orientation;     // IMU orientation (yaw corrected by pose graph)
    Eigen::Vector3f angular_velocity;   // gyro
    
    Eigen::Vector3f position_raw;       // Odometry before pose graph
    bool icp_valid;                     // true when pose graph has keyframes
    float icp_error;                    // 0 when using pose graph
    
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
    
    std::vector<Eigen::Vector2f> getGlobalMap() const;
    std::vector<Eigen::Vector2f> getLatestScan() const;

private:
    void lowStateCallback(const void* message);
    void localizationLoop();
    void publishLoop();
    
    void updateLegOdometry(const std::vector<float>& joint_pos,
                          const std::vector<float>& joint_vel,
                          const std::vector<float>& joint_tau,
                          const Eigen::Vector3f& gyro,
                          float dt);
    
    bool detectSquat(const std::vector<float>& joint_vel) const;
    std::vector<Eigen::Vector2f> getLidarScan2D();
    std::vector<Eigen::Vector2f> generateMap();
    void fuseLidarPosition(const Eigen::Vector2f& lidar_pos,
                          float lidar_yaw,
                          bool icp_success);
    
    LocalizationState state_;
    mutable std::mutex state_mutex_;
    
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> low_state_sub_;
    std::unique_ptr<unitree::robot::ChannelPublisher<LocalizationState>> state_pub_;
    
    std::unique_ptr<ScanMatcher> matcher_;
    std::unique_ptr<PoseGraph> pose_graph_;
    std::unique_ptr<LivoxInterface> livox_;
    std::unique_ptr<UDPPublisher> udp_publisher_;
    
    std::vector<float> latest_joint_pos_;
    std::vector<float> latest_joint_vel_;
    std::vector<Eigen::Vector2f> latest_scan_2d_;
    
    // Map from pose graph (keyframe scans in optimized poses), cached for getGlobalMap()
    std::vector<Eigen::Vector2f> cached_map_;
    
    std::atomic<bool> running_;
    std::thread localization_thread_;
    std::thread publish_thread_;
    
    std::chrono::steady_clock::time_point last_update_time_;
    std::chrono::steady_clock::time_point last_slam_time_;
    
    Eigen::Vector3f lidar_offset_;
    float step_scale_;
    float position_alpha_;
    float velocity_alpha_;
    float slam_interval_;
    
    std::atomic<bool> is_stationary_;
    
    // Current correction between raw odometry and pose-graph world pose.
    Eigen::Vector3f slam_correction_ = Eigen::Vector3f::Zero();
    
    Eigen::Quaternionf fused_q_;
    float fused_yaw_;
    bool fused_init_;
};

} // namespace g1_localization
