#include "G1Localizer.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <map>
#include <deque>
#include <unordered_map>
#include "ScanMatcher.hpp"
#include "PoseGraph.hpp"

using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

namespace g1_localization {

G1Localizer::G1Localizer(const std::string& network_interface,
                         const std::string& receiver_ip,
                         uint16_t receiver_port)
    : running_(false)
    , is_stationary_(false)
    , step_scale_(0.6f)
    , position_alpha_(0.3f)
    , velocity_alpha_(0.5f)
    , slam_interval_(0.03f) {
    
    lidar_offset_ = Eigen::Vector3f(0.10f, 0.0f, 0.60f);
    state_.position = Eigen::Vector3f(0.0f, 0.0f, 0.793f);
    state_.velocity = Eigen::Vector3f::Zero();
    state_.orientation = Eigen::Quaternionf::Identity();
    state_.angular_velocity = Eigen::Vector3f::Zero();
    state_.position_raw = state_.position;
    state_.icp_valid = false;
    state_.icp_error = 0.f;
    state_.timestamp_us = 0;
    slam_correction_ = Eigen::Vector3f::Zero();
    fused_init_ = false;
    fused_yaw_ = 0.0f;
    fused_q_ = Eigen::Quaternionf::Identity();
    matcher_ = std::make_unique<ScanMatcher>();
    pose_graph_ = std::make_unique<PoseGraph>();
    pose_graph_->setKeyframeDistanceThreshold(0.35f, 0.25f);
    livox_ = std::make_unique<LivoxInterface>();
    udp_publisher_ = std::make_unique<UDPPublisher>(receiver_ip, receiver_port);
    
    latest_joint_pos_.resize(29, 0.0f);
    latest_joint_vel_.resize(29, 0.0f);
    
    ChannelFactory::Instance()->Init(0, network_interface);
    low_state_sub_ = std::make_unique<ChannelSubscriber<LowState_>>("rt/lowstate");
    low_state_sub_->InitChannel([this](const void* msg) { this->lowStateCallback(msg); }, 10);
    
    std::cout << "\n===============================================" << std::endl;
    std::cout << "[G1Localizer] VERSION 7.0 - CLOSED-LOOP POSE GRAPH" << std::endl;
    std::cout << "===============================================\n" << std::endl;
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
    
    // Stationary check (same thresholds as updateLegOdometry) — when still, do NOT integrate gyro (bias causes yaw drift)
    float total_joint_vel = 0;
    for (int i = 0; i < 12; ++i) total_joint_vel += std::abs(j_vel[i]);
    float gyro_mag = std::sqrt(gyro.x()*gyro.x() + gyro.y()*gyro.y() + gyro.z()*gyro.z());
    bool stationary_for_yaw = (total_joint_vel < 0.2f && gyro_mag < 0.02f);
    
    // YAW STABILIZATION (Complementary Filter)
    // When STATIONARY: trust IMU yaw only (no gyro integration) to eliminate yaw drift from gyro bias.
    // When moving: integrate gyro for responsiveness, correct slowly toward IMU.
    float imu_yaw = std::atan2(2.0f*(q.w()*q.z() + q.x()*q.y()), 
                              1.0f - 2.0f*(q.y()*q.y() + q.z()*q.z()));
    
    if (!fused_init_) {
        fused_yaw_ = imu_yaw;
        fused_init_ = true;
    } else if (stationary_for_yaw) {
        // Robot is still: no gyro integration — lock yaw to IMU to prevent drift
        fused_yaw_ = imu_yaw;
    } else {
        // 1. Prediction (Integrate Gyro)
        fused_yaw_ += gyro.z() * dt;
        
        // 2. Normalize
        while(fused_yaw_ > M_PI) fused_yaw_ -= 2*M_PI;
        while(fused_yaw_ < -M_PI) fused_yaw_ += 2*M_PI;
        
        // 3. Correction (Trust IMU for long-term drift)
        float diff = imu_yaw - fused_yaw_;
        if (diff > M_PI) diff -= 2*M_PI;
        if (diff < -M_PI) diff += 2*M_PI;
        
        fused_yaw_ += 0.05f * diff;
    }

    // Reconstruction of "Stable" Quaternion
    // We keep the IMU's Roll/Pitch (Leveling) but use our Fused Yaw
    float roll = std::atan2(2.0f*(q.w()*q.x() + q.y()*q.z()), 
                           1.0f - 2.0f*(q.x()*q.x() + q.y()*q.y()));
    float pitch = std::asin(2.0f*(q.w()*q.y() - q.z()*q.x()));
    
    Eigen::Quaternionf stable_q = 
        Eigen::AngleAxisf(fused_yaw_, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.orientation = stable_q;
        state_.angular_velocity = gyro;
    }

    updateLegOdometry(j_pos, j_vel, j_tau, gyro, dt);
    
    static auto last_udp = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_udp).count() >= 20) {
        auto s = getState();
        std::vector<Eigen::Vector2f> scan_copy;
        {
             std::lock_guard<std::mutex> lock(state_mutex_);
             scan_copy = latest_scan_2d_;
        }
        udp_publisher_->sendState(s.position_raw, s.position, Eigen::Vector2f(s.velocity.x(), s.velocity.y()), j_pos, j_vel, s.orientation, s.angular_velocity, scan_copy);
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

    // Zero-Velocity Detection: Lock position if robot is stationary
    // Check joint velocities and gyro to detect standing still
    float total_joint_vel = 0;
    for(int i=0; i<12; ++i) {  // Sum leg joint velocities
        total_joint_vel += std::abs(j_vel[i]);
    }
    float gyro_mag = std::sqrt(gyro.x()*gyro.x() + gyro.y()*gyro.y() + gyro.z()*gyro.z());
    
    // Strict Stand-Still Detection
    // If robot is nearly stationary, FORCE velocities to zero and SKIP integration
    if (total_joint_vel < 0.2f && gyro_mag < 0.02f) { // Lowered thresholds (was 0.5/0.05)
        // Robot is standing still - Lock everything
        is_stationary_ = true;
        return;
    }
    is_stationary_ = false;

    if (std::abs(f_v) < 0.10f) f_v = 0.0f; // Deadband 10cm/s (Aggressive)
    if (std::abs(l_v) < 0.10f) l_v = 0.0f;

    float c = std::cos(yaw), s = std::sin(yaw);
    // Integration (Prediction Step) - Pure Odometry acts on position_raw
    state_.position_raw.x() += (f_v * c - l_v * s) * dt;
    state_.position_raw.y() += (f_v * s + l_v * c) * dt;
    
    // Final Position = Odometry + SLAM Correction
    state_.position = state_.position_raw + slam_correction_;
}

void G1Localizer::localizationLoop() {
    std::cout << "[G1Localizer] Pose graph (closed-loop) enabled." << std::endl;

    auto next_tick = std::chrono::steady_clock::now();
    // Higher frequency loop for smoother IMU updates
    const std::chrono::milliseconds tick_duration(10); // 100Hz

    while (running_) {
        auto now = std::chrono::steady_clock::now();
        next_tick += tick_duration;
        
        // 1. Get State Snapshot (Thread Safe)
        Eigen::Quaternionf q_raw;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            q_raw = state_.orientation;
        }

        // LIO-Livox style: use LiDAR IMU for scan stabilization (deskew is done in LivoxInterface),
        // and use the LiDAR IMU's roll/pitch for leveling if available.
        Eigen::Quaternionf q_level = q_raw;
        if (livox_->hasImuOrientation()) {
            q_level = livox_->getLatestImuOrientation();
        }

        // Pre-calculate rotation matrices at this state
        Eigen::Matrix3f R_body = q_level.normalized().toRotationMatrix();
        float base_yaw = std::atan2(R_body(1,0), R_body(0,0));
        
        // Correct Leveling Transform:
        // We want vectors in a frame that is Yaw-Aligned but Gravity-Leveled
        Eigen::Matrix3f R_yaw = Eigen::AngleAxisf(base_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        // R_level transforms BODY -> LEVELED frame
        Eigen::Matrix3f R_level_transform = R_yaw.transpose() * R_body;

        auto points_3d = livox_->getLatestPointCloud();
        std::vector<Eigen::Vector2f> base_frame_scan;
        
        // 1. Scan Compression (Ground to Lidar Height)
        // Goal: Project all points from ground up to LiDAR height onto a 2D plane.
        // This ensures we catch low obstacles while maintaining a fast 2D scan for ICP.
        
        // Removed: const float scan_grid = 0.05f; // Now using adaptive grid
        std::unordered_map<uint64_t, Eigen::Vector2f> compressed_scan;
        int n_points = points_3d.size();
        Eigen::Vector3f flat_offset(0.10f, 0.0f, 0.0f); // LiDAR is 10cm forward

        for (int i = 0; i < n_points; ++i) {
            Eigen::Vector3f p_lidar = points_3d[i];
            
            // 2. Transform to Body Frame
            Eigen::Vector3f p_body = p_lidar + flat_offset;
            
            // 3. Leveling
            Eigen::Vector3f p_level = R_level_transform * p_body;

            // 4. Compression Filter: Ground (~-0.6m) to LiDAR base (~0.0m)
            // floor_offset = -0.45m (catch everything > 15cm above ground)
            if (p_level.z() < -0.45f || p_level.z() > 0.10f) continue;
            
            // 5. Body Filtering (XY)
            float dist_xy = std::sqrt(p_level.x() * p_level.x() + p_level.y() * p_level.y());
            
            // 15.0m Long Range vision (Requested for large rooms)
            if (dist_xy > 15.0f) continue; 

            // Refined Robot Body Filter:
            // The LiDAR is at x=0.1. Behind (x < 0.1) we filter more broadly.
            // In front (x > 0.1) we only filter very close to the lens.
            if (p_level.x() < 0.15f) {
                if (dist_xy < 0.35f) continue; // Core body
            } else {
                if (dist_xy < 0.15f) continue; // Protective lens/casing
            }
            
            // Remove the old aggressive 0.5m filter that was killing curbs
            
            // 6. Adaptive Grid Deduplication (Distance-based resolution)
            float adaptive_grid;
            if (dist_xy < 1.0f) {
                adaptive_grid = 0.02f;  // 2cm Close
            } else if (dist_xy < 3.0f) {
                adaptive_grid = 0.05f;  // 5cm Mid
            } else if (dist_xy < 7.0f) {
                adaptive_grid = 0.10f;  // 10cm Far
            } else {
                adaptive_grid = 0.20f;  // 20cm Very Far
            }
            
            int32_t ix = static_cast<int32_t>(std::lround(p_level.x() / adaptive_grid));
            int32_t iy = static_cast<int32_t>(std::lround(p_level.y() / adaptive_grid));
            uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) | static_cast<uint32_t>(iy);
            
            if (compressed_scan.find(key) == compressed_scan.end()) {
                compressed_scan.emplace(key, Eigen::Vector2f(p_level.x(), p_level.y()));
            }
        }

        for (auto const& [key, pt] : compressed_scan) {
            base_frame_scan.push_back(pt);
        }
        
        // THREAD SAFE UPDATE:
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_scan_2d_.clear();
            for (const auto& pt : base_frame_scan) {
                 if (latest_scan_2d_.size() >= 1000) break;
                 latest_scan_2d_.push_back(pt);
            }
        }
        
        if (n_points > 0 && base_frame_scan.empty()) { // Check base_frame_scan instead
             // printf("[G1Localizer] WARNING: Input %d points, but Output 0! Filter killed all?\n", n_points);
        }

        /*
        static auto last_log = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log).count() >= 5000) {
             // Removed for performance
        }
        */

        // Odometry pose for pose graph (position_raw from leg odometry, yaw from IMU)
        float odom_x, odom_y, odom_theta;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            odom_x = state_.position_raw.x();
            odom_y = state_.position_raw.y();
        }
        odom_theta = std::atan2(2.0f*(q_raw.w()*q_raw.z() + q_raw.x()*q_raw.y()),
                               1.0f - 2.0f*(q_raw.y()*q_raw.y() + q_raw.z()*q_raw.z()));

        // Pose graph: add keyframe when moved enough, run loop closure, optimize
        if (pose_graph_->shouldAddKeyframe(odom_x, odom_y, odom_theta) && base_frame_scan.size() > 80) {
            pose_graph_->addKeyframe(odom_x, odom_y, odom_theta, base_frame_scan);
            pose_graph_->tryLoopClosures(matcher_.get(), 15, 0.18f);
            pose_graph_->optimize(20);
        }

        float wx, wy, wtheta;
        pose_graph_->getCurrentPoseInWorld(odom_x, odom_y, odom_theta, &wx, &wy, &wtheta);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            slam_correction_.x() = wx - odom_x;
            slam_correction_.y() = wy - odom_y;
            slam_correction_.z() = 0.f;
            state_.position.x() = wx;
            state_.position.y() = wy;
            state_.icp_valid = (pose_graph_->numKeyframes() > 0);
            state_.icp_error = 0.f;
            float roll = std::atan2(2.0f*(q_raw.w()*q_raw.x() + q_raw.y()*q_raw.z()),
                                   1.0f - 2.0f*(q_raw.x()*q_raw.x() + q_raw.y()*q_raw.y()));
            float pitch = std::asin(2.0f*(q_raw.w()*q_raw.y() - q_raw.z()*q_raw.x()));
            state_.orientation = Eigen::AngleAxisf(wtheta, Eigen::Vector3f::UnitZ()) *
                                Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                                Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
            cached_map_ = pose_graph_->getMapPoints();
        }

        Eigen::Vector2f broadcast_pos(wx, wy);
        float broadcast_yaw = wtheta;

        // 5. LIVE BROADCASTS — points in world frame to match fixed map
        static auto last_live_send = std::chrono::steady_clock::now();
        static std::deque<std::pair<std::chrono::steady_clock::time_point, Eigen::Vector2f>> viz_window;
        static auto last_viz_publish = std::chrono::steady_clock::now();
        static std::vector<Eigen::Vector2f> last_good_snapshot;
        
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_live_send).count() >= 50) {
            if (!base_frame_scan.empty()) {
                float c = std::cos(broadcast_yaw);
                float s = std::sin(broadcast_yaw);
                std::vector<Eigen::Vector2f> live_pts;
                size_t stride = (base_frame_scan.size() + 999) / 1000;
                for(size_t i=0; i<base_frame_scan.size(); i+=stride) {
                    live_pts.emplace_back(base_frame_scan[i].x() * c - base_frame_scan[i].y() * s + broadcast_pos.x(),
                                          base_frame_scan[i].x() * s + base_frame_scan[i].y() * c + broadcast_pos.y());
                }

                // Accumulate this chunk
                for (const auto& p : live_pts) {
                    viz_window.emplace_back(now, p);
                }
            }

            // Keep last 2.5s of points for full 360 coverage
            const int window_ms = 5000;
            while (!viz_window.empty() &&
                   std::chrono::duration_cast<std::chrono::milliseconds>(now - viz_window.front().first).count() > window_ms) {
                viz_window.pop_front();
            }

            // Publish snapshot at 2Hz
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_viz_publish).count() >= 500) {
                last_viz_publish = now;

                std::vector<Eigen::Vector2f> snapshot;
                
                // Try to build snapshot from window
                if (!viz_window.empty()) {
                    // Grid dedupe (3cm) for higher fidelity
                    const float grid = 0.03f;
                    std::unordered_map<uint64_t, Eigen::Vector2f> uniq;
                    uniq.reserve(3000);
                    const size_t max_points = 2500;

                    for (const auto& item : viz_window) {
                        const auto& pt = item.second;
                        int32_t ix = static_cast<int32_t>(std::lround(pt.x() / grid));
                        int32_t iy = static_cast<int32_t>(std::lround(pt.y() / grid));
                        uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
                                       static_cast<uint32_t>(iy);
                        if (uniq.find(key) == uniq.end()) {
                            uniq.emplace(key, pt);
                            if (uniq.size() >= max_points) break;
                        }
                    }

                    snapshot.reserve(uniq.size());
                    for (const auto& kv : uniq) snapshot.push_back(kv.second);
                    
                    // If deduplication removed everything, use raw window data
                    if (snapshot.empty() && !viz_window.empty()) {
                        snapshot.reserve(std::min(viz_window.size(), size_t(1500)));
                        for (const auto& item : viz_window) {
                            snapshot.push_back(item.second);
                            if (snapshot.size() >= 1500) break;
                        }
                    }
                }
                
                // Always publish something: current snapshot, or last good snapshot
                if (!snapshot.empty()) {
                    last_good_snapshot = snapshot;  // Remember this good snapshot
                    udp_publisher_->sendMap(snapshot);
                } else if (!last_good_snapshot.empty()) {
                    // Window is empty but we have a previous good snapshot - keep sending it
                    udp_publisher_->sendMap(last_good_snapshot);
                }
                // If both are empty, don't send anything (first startup)
            }
            last_live_send = now;
        }

        { std::lock_guard<std::mutex> lock(state_mutex_); latest_scan_2d_ = base_frame_scan; }
        
        Eigen::Vector3f pose_pos(wx, wy, state_.position.z());
        float roll_b = std::atan2(2.0f*(q_raw.w()*q_raw.x() + q_raw.y()*q_raw.z()), 1.0f - 2.0f*(q_raw.x()*q_raw.x() + q_raw.y()*q_raw.y()));
        float pitch_b = std::asin(2.0f*(q_raw.w()*q_raw.y() - q_raw.z()*q_raw.x()));
        Eigen::Quaternionf q_pose = Eigen::AngleAxisf(wtheta, Eigen::Vector3f::UnitZ()) *
                                   Eigen::AngleAxisf(pitch_b, Eigen::Vector3f::UnitY()) *
                                   Eigen::AngleAxisf(roll_b, Eigen::Vector3f::UnitX());
        auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        udp_publisher_->sendPose(timestamp_us, pose_pos, q_pose);

        // Precise sleep to maintain 50Hz
        std::this_thread::sleep_until(next_tick);
    }
}


LocalizationState G1Localizer::getState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

std::vector<Eigen::Vector2f> G1Localizer::getGlobalMap() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return cached_map_;
}

std::vector<Eigen::Vector2f> G1Localizer::getLatestScan() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_scan_2d_;
}

} // namespace g1_localization


