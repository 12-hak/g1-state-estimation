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
    
    // Sensor is ~10cm forward of pivot
    lidar_offset_ = Eigen::Vector3f(0.10f, 0.0f, 0.60f);
    slam_correction_ = Eigen::Vector3f::Zero();
    slam_yaw_correction_ = 0.0f;

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
    
    std::cout << "\n===============================================" << std::endl;
    std::cout << "[G1Localizer] VERSION 4.0 - STRICT LOCK + MATH FIX" << std::endl;
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
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.orientation = q;
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
    float last_map_update_x = 0;
    float last_map_update_y = 0;
    
    std::cout << "[G1Localizer] MAP CAP 1000 ENABLED" << std::endl;

    auto next_tick = std::chrono::steady_clock::now();
    const std::chrono::milliseconds tick_duration(20); // 50Hz

    while (running_) {
        auto now = std::chrono::steady_clock::now();
        next_tick += tick_duration;
        
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
        
        // Correct Leveling Transform:
        // We want vectors in a frame that is Yaw-Aligned but Gravity-Leveled
        // P_level = R_yaw^T * P_world
        // P_world = R_body * P_body
        // -> P_level = R_yaw^T * R_body * P_body
        
        Eigen::Matrix3f R_yaw = Eigen::AngleAxisf(base_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
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
            
            // 3.0m visualization/performance limit
            if (dist_xy > 3.5f) continue; 

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
            // Close objects (<1m): 2cm grid for high sensitivity
            // Mid-range (1-2m): 5cm grid
            // Far objects (>2m): 10cm grid
            float adaptive_grid;
            if (dist_xy < 1.0f) {
                adaptive_grid = 0.02f;  // 2cm for close obstacles
            } else if (dist_xy < 2.0f) {
                adaptive_grid = 0.05f;  // 5cm for mid-range
            } else {
                adaptive_grid = 0.10f;  // 10cm for far objects
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

        // 2. Local Map Pruning (Performance Limit)
        // Reduced to 3.5m to keep the visualizer fast as requested.
        const float map_radius = 3.5f; 
        const float map_radius_sq = map_radius * map_radius;
        
        if (!global_map_.empty()) {
            int pruned_count = 0;
            auto it = global_map_.begin();
            while (it != global_map_.end()) {
                float dx = it->pt.x() - base_pos.x();
                float dy = it->pt.y() - base_pos.y();
                float dist_sq = dx*dx + dy*dy;
                
                if (dist_sq > map_radius_sq) {
                    it = global_map_.erase(it);
                    pruned_count++;
                } else {
                    ++it;
                }
            }
            
            if (pruned_count > 0) {
                std::cout << "[G1Localizer] Pruned " << pruned_count << " old points. Map size: " << global_map_.size() << std::endl;
            }
        }
        
        // MAP UPDATE MOVED TO AFTER ICP (See below)
        
        // Get current corrected position and yaw for broadcast (before ICP, uses current state)
        Eigen::Vector3f corrected_pos_broadcast;
        float corrected_yaw_broadcast;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            corrected_pos_broadcast = state_.position;
            float raw_yaw = std::atan2(2.0f*(q.w()*q.z() + q.x()*q.y()), 
                                     1.0f - 2.0f*(q.y()*q.y() + q.z()*q.z()));
            corrected_yaw_broadcast = raw_yaw + slam_yaw_correction_;
        }

        // 5. LIVE BROADCASTS (Moved BEFORE ICP so it always runs, even when stationary)
        //
        // Always publish an accumulated "360" snapshot (deduped) at ~2Hz.
        // Accumulates points over 2.5s window and publishes complete scans.
        static auto last_live_send = std::chrono::steady_clock::now();
        static std::deque<std::pair<std::chrono::steady_clock::time_point, Eigen::Vector2f>> viz_window;
        static auto last_viz_publish = std::chrono::steady_clock::now();
        static std::vector<Eigen::Vector2f> last_good_snapshot;  // Keep last good snapshot
        
        // Accumulate points every 50ms
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_live_send).count() >= 50) {
            // Accumulate points if we have them
            if (!base_frame_scan.empty()) {
                float c = std::cos(corrected_yaw_broadcast);
                float s = std::sin(corrected_yaw_broadcast);
                std::vector<Eigen::Vector2f> live_pts;
                // High density (up to 1000 points)
                size_t stride = (base_frame_scan.size() + 999) / 1000;
                for(size_t i=0; i<base_frame_scan.size(); i+=stride) {
                    live_pts.emplace_back(base_frame_scan[i].x() * c - base_frame_scan[i].y() * s + corrected_pos_broadcast.x(),
                                          base_frame_scan[i].x() * s + base_frame_scan[i].y() * c + corrected_pos_broadcast.y());
                }

                // Accumulate this chunk
                for (const auto& p : live_pts) {
                    viz_window.emplace_back(now, p);
                }
            }

            // Keep last 2.5s of points for full 360 coverage
            const int window_ms = 2500;
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
                    // Grid dedupe (5cm) to avoid dense blobs and stay under UDP limits
                    const float grid = 0.05f;
                    std::unordered_map<uint64_t, Eigen::Vector2f> uniq;
                    uniq.reserve(2000);
                    const size_t max_points = 1500;

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

        // 3. Point-to-Line Registration
        if (!global_map_.empty() && base_frame_scan.size() > 50) {
            // STATIONARY CHECK:
            // If the robot is locked (stationary), DO NOT RUN ICP.
            // This prevents "jumping" when standing still against a wall.
            if (is_stationary_) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                state_.icp_valid = true; // Trust the lock
                
                // Throttle "Stationary" log
                static auto last_stat_log = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stat_log).count() >= 2000) {
                    std::cout << "[G1Localizer] STATIONARY - Skipping ICP to prevent jump." << std::endl;
                    last_stat_log = now;
                }
                // Skip ICP when stationary - broadcast already happened above
            } else {
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
                
                // Debug ICP results every 2 seconds
                static auto last_icp_log = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_icp_log).count() >= 2000) {
                    std::cout << "[G1Localizer] ICP: converged=" << result.converged 
                              << ", error=" << result.error << "m" << std::endl;
                    last_icp_log = now;
                }
                
                // Tightened threshold: 0.20m (Balanced)
                if (result.converged && result.error < 0.20f) {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    
                    // 1. Position Correction
                    Eigen::Vector2f diff = result.translation - state_.position_raw.head<2>();
                    Eigen::Vector3f target_correction(diff.x(), diff.y(), 0.0f);
                    
                    // 2. Yaw Correction (New!)
                    float icp_yaw = std::atan2(result.rotation(1,0), result.rotation(0,0));
                    float raw_yaw = std::atan2(2.0f*(q.w()*q.z() + q.x()*q.y()), 
                                             1.0f - 2.0f*(q.y()*q.y() + q.z()*q.z()));
                    float yaw_diff = icp_yaw - raw_yaw;
                    // Normalize angle
                    if (yaw_diff > M_PI) yaw_diff -= 2*M_PI;
                    if (yaw_diff < -M_PI) yaw_diff += 2*M_PI;

                    // Smoothly update corrections
                    float alpha = 0.5f;
                    slam_correction_ = slam_correction_ * (1.0f - alpha) + target_correction * alpha;
                    slam_yaw_correction_ = slam_yaw_correction_ * (1.0f - alpha) + yaw_diff * alpha;
                    
                    // Apply to state
                    state_.position = state_.position_raw + slam_correction_;
                    state_.icp_valid = true;
                    state_.icp_error = result.error;
                } else {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    state_.icp_valid = false;
                }
            }
        }


        // 4. MAP UPDATE STEP (Performed AFTER ICP Correction)
        // Get latest corrected position and yaw
        Eigen::Vector3f corrected_pos;
        float corrected_yaw;
        bool icp_ok = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            corrected_pos = state_.position;
            icp_ok = state_.icp_valid;
            float raw_yaw = std::atan2(2.0f*(q.w()*q.z() + q.x()*q.y()), 
                                     1.0f - 2.0f*(q.y()*q.y() + q.z()*q.z()));
            corrected_yaw = raw_yaw + slam_yaw_correction_;
        }

        float dist_moved = std::sqrt(std::pow(corrected_pos.x() - last_map_update_x, 2) + 
                                   std::pow(corrected_pos.y() - last_map_update_y, 2));

        // STATIONARY UPDATE LOGIC (As requested: Update every 1s when standing)
        static auto last_stationary_map_update = std::chrono::steady_clock::now();
        bool stationary_update_ready = is_stationary_ && 
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stationary_map_update).count() >= 1000;

        // Trigger Map Update if:
        // 1. Moving (>0.2m) and ICP is valid
        // 2. Stationary Refresh (Every 1.0s)
        // 3. First scan
        if (global_map_.empty() || (dist_moved > 0.2f && icp_ok) || stationary_update_ready) {
            
            float c = std::cos(corrected_yaw); // Use CORRECTED YAW
            float s = std::sin(corrected_yaw);
            
            if (stationary_update_ready) {
                last_stationary_map_update = now;
                // CLEAR proximal map (2m) for a fresh draw
                auto it = global_map_.begin();
                while(it != global_map_.end()) {
                    if ((it->pt - corrected_pos.head<2>()).norm() < 2.0f) {
                        it = global_map_.erase(it);
                    } else ++it;
                }
            }
            
            std::vector<Eigen::Vector2f> new_world_points;
            for(const auto& p : base_frame_scan) {
                // Use CORRECTED position and CORRECTED yaw for map building
                float wx = p.x() * c - p.y() * s + corrected_pos.x();
                float wy = p.x() * s + p.y() * c + corrected_pos.y();
                new_world_points.emplace_back(wx, wy);
            }

            auto new_structure = ScanMatcher::computeStructure(new_world_points);
            
            int points_added = 0;
            for(const auto& kp : new_structure) {
                bool close = false;
                int check_range = std::min((int)global_map_.size(), 1000);
                for(int j=0; j<check_range; ++j) {
                    int idx = global_map_.size() - 1 - j;
                    if ((global_map_[idx].pt - kp.pt).squaredNorm() < 0.01f) { // 10cm grid
                        close = true; 
                        break; 
                    }
                }
                if (!close) {
                    global_map_.push_back(kp);
                    points_added++;
                }
            }
            
            // Map update updated secretly in background
            last_map_update_x = corrected_pos.x();
            last_map_update_y = corrected_pos.y();
        }

        { std::lock_guard<std::mutex> lock(state_mutex_); latest_scan_2d_ = base_frame_scan; }
        
        // Broadcast pose for recorder and viz
        auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        udp_publisher_->sendPose(timestamp_us, corrected_pos, q);

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
    std::vector<Eigen::Vector2f> map;
    map.reserve(global_map_.size());
    for(const auto& p : global_map_) {
        map.push_back(p.pt);
    }
    return map;
}

std::vector<Eigen::Vector2f> G1Localizer::getLatestScan() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_scan_2d_;
}

} // namespace g1_localization

} // namespace g1_localization


