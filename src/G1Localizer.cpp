#include "G1Localizer.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <map>
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
    
    // Extrinsics Upgrade (Aggressive Circle Fix):
    // Based on empirical observation of 20cm circle.
    // X: +0.20m (20cm Forward of pivot)
    // Z: +0.60m (60cm Up)
    lidar_offset_ = Eigen::Vector3f(0.20f, 0.0f, 0.60f);
    slam_correction_ = Eigen::Vector3f::Zero();

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
    mapping_thread_ = std::thread(&G1Localizer::mappingThreadLoop, this);
    std::cout << "[G1Localizer] Localization + Mapping threads started" << std::endl;
}

void G1Localizer::stop() {
    running_ = false;
    if (localization_thread_.joinable()) localization_thread_.join();
    if (mapping_thread_.joinable()) mapping_thread_.join();
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
            
            // 1. Motion De-skewing + Extrinsics + Tilt Compensation
            float scan_time = 0.1f; 
            int n_points = points_3d.size();
            
            // EXTREME TEST: 
            // Setting to 1.0m to verify if this variable does ANYTHING.
            // "FLAT ROBOT" FIX:
            // Z=0 to prevent tilt error amplification.
            // X=0.10m (Standard Face Offset).
            Eigen::Vector3f flat_offset(0.10f, 0.0f, 0.0f);

            for (int i = 0; i < n_points; ++i) {
                float dt = (float)i / n_points * scan_time; 
                float angle_correction = -gyro.z() * dt;
                Eigen::Matrix3f R_deskew = Eigen::AngleAxisf(angle_correction, Eigen::Vector3f::UnitZ()).toRotationMatrix();
                
                // 1. De-skew point in LiDAR frame
                Eigen::Vector3f p_lidar = R_deskew * points_3d[i];
                
                // 2. Transform to Body Frame
                Eigen::Vector3f p_body = p_lidar + flat_offset;
                
                // 3. Leveling (Standard)
                Eigen::Vector3f p_level = R_tilt.transpose() * p_body;
                
                // DIAGNOSTIC: Remove ALL Filters to see points
                // if (p_level.norm() < 0.2f) continue;
                // if (p_level.z() < -2.0f || p_level.z() > 2.0f) continue;
                
                base_frame_scan.emplace_back(p_level.x(), p_level.y());
                
                // Add to 3D visualization cache (send everything!)
                if (latest_scan_2d_.size() < 1000) { // Limit to 1000 for UDP safety
                     // ERROR: This is unsafe cross-thread access!
                     // latest_scan_2d_.emplace_back(p_level.x(), p_level.y());
                }
            } // End Point Loop
            
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

            // 2. Map Initialization & Dynamic Expansion
            float dist_moved = std::sqrt(std::pow(base_pos.x() - last_map_update_x, 2) + 
                                       std::pow(base_pos.y() - last_map_update_y, 2));
            
            // If checking first frame OR moved enough (Keyframe logic)
            // Reduced to 0.1m for DENSE map building
            if (global_map_.empty() || (dist_moved > 0.1f && state_.icp_valid)) {
                
                // === 3D MAPPING: Process and push to mapping thread ===
                std::vector<Eigen::Vector3f> filtered_scan_3d;
                filtered_scan_3d.reserve(n_points);
                
                // Process each 3D point with proper transformations and filtering
                for (int i = 0; i < n_points; ++i) {
                    float dt = (float)i / n_points * scan_time;
                    float angle_correction = -gyro.z() * dt;
                    Eigen::Matrix3f R_deskew = Eigen::AngleAxisf(angle_correction, Eigen::Vector3f::UnitZ()).toRotationMatrix();
                    
                    // 1. De-skew point in LiDAR frame
                    Eigen::Vector3f p_lidar = R_deskew * points_3d[i];
                    
                    // 2. Transform to Body Frame
                    Eigen::Vector3f p_body = p_lidar + flat_offset;
                    
                    // 3. Leveling to get gravity-aligned point
                    Eigen::Vector3f p_level = R_tilt.transpose() * p_body;
                    
                    // === FILTERING ===
                    // Remove robot body parts (head supports, torso)
                    float dist_xy = std::sqrt(p_level.x() * p_level.x() + p_level.y() * p_level.y());
                    
                    // Filter 1: Remove points too close (robot body)
                    if (dist_xy < 0.3f) continue;  // 30cm radius around robot
                    
                    // Filter 2: Remove points at head height (support structures)
                    if (p_level.z() > -0.5f && p_level.z() < 0.5f && dist_xy < 0.5f) continue;
                    
                    // Filter 3: Remove extreme outliers
                    if (dist_xy > 20.0f) continue;  // 20m max range
                    if (p_level.z() < -3.0f || p_level.z() > 3.0f) continue;  // Reasonable height
                    
                    filtered_scan_3d.push_back(p_level);
                }
                
                uint64_t timestamp_snapshot;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    latest_scan_3d_ = filtered_scan_3d;
                    timestamp_snapshot = state_.timestamp_us;
                }
                
                // Use ICP-corrected pose (base_pos already includes SLAM correction)
                MappingData mapping_data;
                mapping_data.position = base_pos;  // This is position_raw + slam_correction
                mapping_data.orientation = q;
                mapping_data.scan_3d = filtered_scan_3d;  // Filtered points
                mapping_data.timestamp_us = timestamp_snapshot;
                
                if (!tryPushMappingData(mapping_data)) {
                    // Queue full - frame dropped (expected behavior for leaky queue)
                }
                // === END 3D MAPPING ===
                
                // ... (Coordinate transform omitted, keeping existing logic) ...
                // Re-calculating new_world_points logic is inside the block...
                // I need to target the threshold line specifically or include the block.
                
                float c = std::cos(base_yaw);
                float s = std::sin(base_yaw);
                
                std::vector<Eigen::Vector2f> new_world_points;
                for(const auto& p : base_frame_scan) {
                    float wx = p.x() * c - p.y() * s + base_pos.x();
                    float wy = p.x() * s + p.y() * c + base_pos.y();
                    new_world_points.emplace_back(wx, wy);
                }

                auto new_structure = ScanMatcher::computeStructure(new_world_points);
                
                int points_added = 0;
                for(const auto& kp : new_structure) {
                    bool close = false;
                    int check_range = std::min((int)global_map_.size(), 1000); // Increased check range
                    for(int j=0; j<check_range; ++j) {
                        int idx = global_map_.size() - 1 - j;
                        // Reduced to 5cm grid (0.05 * 0.05 = 0.0025)
                        if ((global_map_[idx].pt - kp.pt).squaredNorm() < 0.0025f) { 
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
                    // std::cout << "[G1Localizer] Keyframe added: " << points_added << " points. Map Size: " << global_map_.size() << std::endl;
                    
                    // Send updated map to viz (throttled)
                    std::vector<Eigen::Vector2f> viz_pts;
                    // Cap at 1000 points (12KB) - ULTRA SAFE
                    // Correct Stride Calculation (Ceil)
                    size_t stride = (global_map_.size() + 999) / 1000;
                    for(size_t i=0; i<global_map_.size(); i+=stride) viz_pts.push_back(global_map_[i].pt);
                    udp_publisher_->sendMap(viz_pts);
                    
                    last_map_update_x = base_pos.x();
                    last_map_update_y = base_pos.y();
                }
            }
            
            // Periodic Map Re-Send (Heartbeat) - Ensures late-joining visualizer gets map
            static auto last_map_send = std::chrono::steady_clock::now();
            if (!global_map_.empty() && 
                std::chrono::duration_cast<std::chrono::milliseconds>(now - last_map_send).count() >= 3000) {
                
                std::vector<Eigen::Vector2f> viz_pts;
                // Cap at 1000 points (12KB) - ULTRA SAFE
                // Correct Stride Calculation (Ceil)
                size_t stride = (global_map_.size() + 999) / 1000;
                for(size_t i=0; i<global_map_.size(); i+=stride) viz_pts.push_back(global_map_[i].pt);
                udp_publisher_->sendMap(viz_pts);
                last_map_send = now;
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
                    
                    // DECOUPLED UPDATE:
                    // We trust Leg Odometry (position_raw) for high-frequency smoothness.
                    // We use SLAM to calculate the "Drift Correction".
                    // Target Correction = SLAM_Pose - Odom_Pose (2D Only)
                    Eigen::Vector2f diff = result.translation - state_.position_raw.head<2>();
                    Eigen::Vector3f target_correction(diff.x(), diff.y(), 0.0f);
                    
                    // Smoothly update the correction vector (Alpha Filter)
                    // Alpha 0.5 = Faster drift correction (Stronger SLAM)
                    float alpha = 0.5f;
                    slam_correction_ = slam_correction_ * (1.0f - alpha) + target_correction * alpha;
                    
                    // Apply current correction to final position
                    state_.position = state_.position_raw + slam_correction_;
                    
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
        
        static auto last_log_check = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_check).count() >= 5000) {
             // Only print stats every 5 seconds
             // ... Code to print stats ... but I need to inline it or move it since I replaced the loop block?
             // Actually, the previous 'replace' put the log block HIGHER up (lines 201-218).
             // I am editing lines 284-295 here.
             // I need to search for the log block to edit the frequency. 
        }
    }
}


LocalizationState G1Localizer::getState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

bool G1Localizer::tryPushMappingData(const MappingData& data) {
    std::lock_guard<std::mutex> lock(mapping_queue_mutex_);
    
    // Leaky Queue: Drop if full (never block localization)
    if (mapping_queue_.size() >= MAX_MAPPING_QUEUE_SIZE) {
        return false;  // Drop frame
    }
    
    mapping_queue_.push(data);
    return true;
}

void G1Localizer::mappingThreadLoop() {
    std::cout << "[G1Mapper] Thread started (High Resolution Mode)" << std::endl;
    
    // Global 3D Map (Simple Voxel Grid)
    struct VoxelKey {
        int x, y, z;
        bool operator<(const VoxelKey& other) const {
            if (x != other.x) return x < other.x;
            if (y != other.y) return y < other.y;
            return z < other.z;
        }
    };
    
    std::map<VoxelKey, Eigen::Vector3f> global_map_3d;
    const float voxel_size = 0.02f;  // 2cm voxels for high resolution
    
    auto voxelize = [&](const Eigen::Vector3f& pt) -> VoxelKey {
        return {
            static_cast<int>(std::floor(pt.x() / voxel_size)),
            static_cast<int>(std::floor(pt.y() / voxel_size)),
            static_cast<int>(std::floor(pt.z() / voxel_size))
        };
    };
    
    size_t total_scans_processed = 0;
    size_t total_points_received = 0;
    auto last_save = std::chrono::steady_clock::now();
    
    while (running_) {
        MappingData data;
        bool has_data = false;
        
        // Pop from queue (non-blocking)
        {
            std::lock_guard<std::mutex> lock(mapping_queue_mutex_);
            if (!mapping_queue_.empty()) {
                data = std::move(mapping_queue_.front());
                mapping_queue_.pop();
                has_data = true;
            }
        }
        
        if (!has_data) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        
        // Transform scan to world frame
        Eigen::Matrix3f R = data.orientation.toRotationMatrix();
        size_t points_added_this_scan = 0;
        
        for (const auto& pt_local : data.scan_3d) {
            Eigen::Vector3f pt_world = R * pt_local + data.position;
            
            // Add to voxel grid
            VoxelKey key = voxelize(pt_world);
            global_map_3d[key] = pt_world;  // Overwrite (keeps latest)
            points_added_this_scan++;
        }
        
        total_scans_processed++;
        total_points_received += data.scan_3d.size();
        
        // Periodic save (every 5 seconds for more frequent updates)
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_save).count() >= 5) {
            float filter_efficiency = total_points_received > 0 ? 
                (100.0f * global_map_3d.size() / total_points_received) : 0.0f;
            
            std::cout << "[G1Mapper] Scans: " << total_scans_processed 
                      << ", Points Received: " << total_points_received
                      << ", Map Points: " << global_map_3d.size()
                      << " (" << std::fixed << std::setprecision(1) << filter_efficiency << "% unique)"
                      << std::endl;
            
            // Save to PCD file
            std::string filename = "map_3d.pcd";
            std::ofstream file(filename);
            if (file.is_open()) {
                // PCD Header
                file << "# .PCD v0.7 - Point Cloud Data file format\n";
                file << "VERSION 0.7\n";
                file << "FIELDS x y z\n";
                file << "SIZE 4 4 4\n";
                file << "TYPE F F F\n";
                file << "COUNT 1 1 1\n";
                file << "WIDTH " << global_map_3d.size() << "\n";
                file << "HEIGHT 1\n";
                file << "VIEWPOINT 0 0 0 1 0 0 0\n";
                file << "POINTS " << global_map_3d.size() << "\n";
                file << "DATA ascii\n";
                
                // Points
                for (const auto& [key, pt] : global_map_3d) {
                    file << pt.x() << " " << pt.y() << " " << pt.z() << "\n";
                }
                
                file.close();
                std::cout << "[G1Mapper] Saved " << filename 
                          << " (" << (global_map_3d.size() * 12 / 1024) << " KB)" << std::endl;
            }
            
            last_save = now;
        }
    }
    
    std::cout << "[G1Mapper] Thread stopped. Final map: " << global_map_3d.size() 
              << " points from " << total_scans_processed << " scans" << std::endl;
}

} // namespace g1_localization

