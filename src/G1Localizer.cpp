#include "G1Localizer.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <cmath>
#include <iostream>
#include <algorithm>
#include "ScanMatcher.hpp"

using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

namespace g1_localization {

G1Localizer::G1Localizer(const std::string& network_interface,
                         const std::string& receiver_ip,
                         uint16_t receiver_port)
    : running_(false)
    , step_scale_(0.6f)
    , position_alpha_(0.3f)
    , velocity_alpha_(0.5f)
    , slam_interval_(0.03f) {
    
    lidar_offset_ = Eigen::Vector3f(0.05f, 0.0f, 0.15f);

    state_.position = Eigen::Vector3f(0.0f, 0.0f, 0.793f);
    state_.velocity = Eigen::Vector3f::Zero();
    state_.orientation = Eigen::Quaternionf::Identity();
    state_.angular_velocity = Eigen::Vector3f::Zero();
    state_.position_raw = state_.position;
    state_.icp_valid = false;
    state_.timestamp_us = 0;
    
    matcher_ = std::make_unique<ScanMatcher>();
    livox_ = std::make_unique<LivoxInterface>();
    udp_publisher_ = std::make_unique<UDPPublisher>(receiver_ip, receiver_port);
    
    latest_joint_pos_.resize(29, 0.0f);
    latest_joint_vel_.resize(29, 0.0f);
    
    ChannelFactory::Instance()->Init(0, network_interface);
    low_state_sub_ = std::make_unique<ChannelSubscriber<LowState_>>("rt/lowstate");
    low_state_sub_->InitChannel([this](const void* msg) { this->lowStateCallback(msg); }, 10);
    
    std::cout << "[G1Localizer] Professional Mode (FAST-LIO2 Style) Initialized" << std::endl;
}

G1Localizer::~G1Localizer() { stop(); }

void G1Localizer::start() {
    running_ = true;
    livox_->initialize();
    localization_thread_ = std::thread(&G1Localizer::localizationLoop, this);
}

void G1Localizer::stop() {
    running_ = false;
    if (localization_thread_.joinable()) localization_thread_.join();
    livox_->uninitialize();
}

void G1Localizer::lowStateCallback(const void* message) {
    const LowState_* msg = static_cast<const LowState_*>(message);
    auto now = std::chrono::steady_clock::now();
    
    std::vector<float> j_pos(29), j_vel(29), j_tau(29);
    for (int i = 0; i < 29; ++i) {
        j_pos[i] = msg->motor_state()[i].q();
        j_vel[i] = msg->motor_state()[i].dq();
        j_tau[i] = msg->motor_state()[i].tau_est();
    }
    
    Eigen::Vector3f gyro(msg->imu_state().gyroscope()[0], msg->imu_state().gyroscope()[1], msg->imu_state().gyroscope()[2]);
    Eigen::Quaternionf q(msg->imu_state().quaternion()[0], msg->imu_state().quaternion()[1], msg->imu_state().quaternion()[2], msg->imu_state().quaternion()[3]);
    
    float dt = 0.02f;
    if (last_update_time_.time_since_epoch().count() > 0) {
        auto dur = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_time_);
        dt = std::min(static_cast<float>(dur.count()) / 1e6f, 0.1f);
    }
    last_update_time_ = now;
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.orientation = q;
        state_.angular_velocity = gyro;
    }

    updateLegOdometry(j_pos, j_vel, j_tau, gyro, dt);
    
    static auto last_udp = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_udp).count() >= 20) {
        auto s = getState();
        udp_publisher_->sendState(s.position_raw, s.position, Eigen::Vector2f(s.velocity.x(), s.velocity.y()), j_pos, j_vel, s.orientation, s.angular_velocity, latest_scan_2d_);
        last_udp = now;
    }
}

void G1Localizer::updateLegOdometry(const std::vector<float>& j_pos, const std::vector<float>& j_vel, const std::vector<float>& j_tau, const Eigen::Vector3f& gyro, float dt) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    float yaw = 2.0f * std::atan2(state_.orientation.z(), state_.orientation.w());
    float lt = 0, rt = 0;
    for(int i=0; i<6; ++i) { lt += std::abs(j_tau[i]); rt += std::abs(j_tau[i+6]); }
    bool left = lt > rt;
    
    // Leg Odometry Logic
    float f_v = j_vel[left ? 0 : 6] * step_scale_;
    float l_v = -j_vel[left ? 1 : 7] * 0.4f;

    if (std::abs(f_v) < 0.01f) f_v = 0.0f;
    if (std::abs(l_v) < 0.01f) l_v = 0.0f;

    float c = std::cos(yaw), s = std::sin(yaw);
    // Integration (Prediction Step)
    state_.position.x() += (f_v * c - l_v * s) * dt;
    state_.position.y() += (f_v * s + l_v * c) * dt;
    state_.position_raw = state_.position;
}

void G1Localizer::localizationLoop() {
    float last_map_update_x = 0;
    float last_map_update_y = 0;

    while (running_) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_slam_time_).count() >= 30) {
            
            // 1. Get State Snapshot (Thread Safe)
            Eigen::Quaternionf q;
            Eigen::Vector3f base_pos;
            Eigen::Vector3f gyro;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                q = state_.orientation;
                base_pos = state_.position;
                gyro = state_.angular_velocity;
            }

            // Pre-calculate rotation matrices at this state
            Eigen::Matrix3f R_body = q.toRotationMatrix();
            float base_yaw = std::atan2(R_body(1,0), R_body(0,0));
            Eigen::Matrix3f R_tilt = R_body * Eigen::AngleAxisf(base_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix().transpose();

            auto points_3d = livox_->getLatestPointCloud();
            std::vector<Eigen::Vector2f> base_frame_scan;
            
            // Motion De-skewing + Extrinsics + Tilt Compensation
            float scan_time = 0.1f; 
            int n_points = points_3d.size();

            for (int i = 0; i < n_points; ++i) {
                float dt = (float)i / n_points * scan_time; 
                float angle_correction = -gyro.z() * dt;
                Eigen::Matrix3f R_deskew = Eigen::AngleAxisf(angle_correction, Eigen::Vector3f::UnitZ()).toRotationMatrix();
                
                Eigen::Vector3f p_deskewed = R_deskew * points_3d[i];
                Eigen::Vector3f p_in_base = p_deskewed + lidar_offset_;
                Eigen::Vector3f p_level = R_tilt.transpose() * p_in_base;
                
                if (p_level.norm() < 0.2f) continue;
                if (p_level.z() < -0.3f || p_level.z() > 0.3f) continue;
                
                base_frame_scan.emplace_back(p_level.x(), p_level.y());
            }

            // 2. Map Initialization & Dynamic Expansion
            float dist_moved = std::sqrt(std::pow(base_pos.x() - last_map_update_x, 2) + 
                                       std::pow(base_pos.y() - last_map_update_y, 2));
            
            // If checking first frame OR moved enough (Keyframe logic)
            if (global_map_.empty() || (dist_moved > 0.5f && state_.icp_valid)) {
                
                // Convert current scan to World Frame and add to map
                // T_world_scan = T_world_robot * T_robot_scan
                // But our base_frame_scan is ALREADY in T_robot_scan (leveled)
                // So we just need to rotate by Base Yaw and add Base Pos
                float c = std::cos(base_yaw); // Note: ideally use the ICP corrected yaw if available
                float s = std::sin(base_yaw);
                if (state_.icp_valid) { 
                    // Use corrected pose for mapping to avoid drift
                    // But wait, base_pos is ALREADY the corrected pose from the last iteration + odometry
                }

                std::vector<Eigen::Vector2f> new_world_points;
                for(const auto& p : base_frame_scan) {
                    float wx = p.x() * c - p.y() * s + base_pos.x();
                    float wy = p.x() * s + p.y() * c + base_pos.y();
                    new_world_points.emplace_back(wx, wy);
                }

                // Compute structure (normals) for new points
                auto new_structure = ScanMatcher::computeStructure(new_world_points);
                
                // Add to global map (Simple Voxel filter to avoid duplicates)
                // We only add points if they are far from existing points
                int points_added = 0;
                for(const auto& kp : new_structure) {
                    bool close = false;
                    // Check against recent points (optimization: check last 500)
                    int check_range = std::min((int)global_map_.size(), 500);
                    for(int j=0; j<check_range; ++j) {
                        int idx = global_map_.size() - 1 - j;
                        if ((global_map_[idx].pt - kp.pt).squaredNorm() < 0.04f) { // 20cm grid
                            close = true; 
                            break; 
                        }
                    }
                    if (!close) {
                        global_map_.push_back(kp);
                        points_added++;
                    }
                }
                
                if (points_added > 0) {
                    std::cout << "[G1Localizer] Keyframe added: " << points_added << " points. Map Size: " << global_map_.size() << std::endl;
                    
                    // Send updated map to viz (throttled)
                    std::vector<Eigen::Vector2f> viz_pts;
                    // Decimate for network speed
                    for(size_t i=0; i<global_map_.size(); i+=2) viz_pts.push_back(global_map_[i].pt);
                    udp_publisher_->sendMap(viz_pts);
                    
                    last_map_update_x = base_pos.x();
                    last_map_update_y = base_pos.y();
                }
            }

            // 3. Point-to-Line Registration
            if (!global_map_.empty() && base_frame_scan.size() > 50) {
                std::vector<Eigen::Vector2f> sub_scan;
                if (base_frame_scan.size() > 300) {
                    for(size_t i=0; i<base_frame_scan.size(); i+=2) sub_scan.push_back(base_frame_scan[i]);
                } else sub_scan = base_frame_scan;

                // CLONE METHOD: Use Predicted State as Guess
                Eigen::Vector2f predicted_pos;
                float predicted_yaw;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    predicted_pos = Eigen::Vector2f(state_.position.x(), state_.position.y());
                    // Use IMU yaw + Odo Integration as guess
                    predicted_yaw = std::atan2(2.0f*(q.w()*q.z() + q.x()*q.y()), 
                                             1.0f - 2.0f*(q.y()*q.y() + q.z()*q.z()));
                }

                auto result = matcher_->align(sub_scan, global_map_, predicted_pos, predicted_yaw);
                
                if (result.converged && result.error < 0.15f) {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    state_.position.x() = result.translation.x();
                    state_.position.y() = result.translation.y();
                    state_.icp_valid = true;
                    state_.icp_error = result.error;
                } else {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    state_.icp_valid = false;
                }
            }
            { std::lock_guard<std::mutex> lock(state_mutex_); latest_scan_2d_ = base_frame_scan; }
            last_slam_time_ = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

LocalizationState G1Localizer::getState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

} // namespace g1_localization
