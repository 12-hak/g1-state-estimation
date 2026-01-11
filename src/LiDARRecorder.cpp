#include "LiDARRecorder.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cstring>
#include <unistd.h>

using namespace g1_localization;

LiDARRecorder::LiDARRecorder(const std::string& output_dir,
                             const std::string& localizer_ip,
                             uint16_t localizer_port)
    : output_dir_(output_dir)
    , running_(false)
    , frame_count_(0)
    , pose_socket_(-1) {
    
    livox_ = std::make_unique<LivoxInterface>();
    
    // Create output directory if needed
    system(("mkdir -p " + output_dir).c_str());
    
    // Generate timestamp-based filename
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << output_dir << "/recording_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    std::string base_filename = ss.str();
    
    // Open output files
    pose_file_.open(base_filename + "_poses.txt");
    cloud_file_.open(base_filename + "_clouds.bin", std::ios::binary);
    
    if (!pose_file_.is_open() || !cloud_file_.is_open()) {
        throw std::runtime_error("Failed to open output files");
    }
    
    // Write pose file header
    pose_file_ << "# timestamp_us x y z qw qx qy qz\n";
    
    std::cout << "[Recorder] Output: " << base_filename << std::endl;
    
    // Setup UDP socket to receive pose from localizer
    pose_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (pose_socket_ < 0) {
        throw std::runtime_error("Failed to create pose socket");
    }
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(localizer_port);
    
    if (bind(pose_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        throw std::runtime_error("Failed to bind pose socket");
    }
    
    // Set socket timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms
    setsockopt(pose_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    latest_pose_.valid = false;
}

LiDARRecorder::~LiDARRecorder() {
    stop();
    if (pose_file_.is_open()) pose_file_.close();
    if (cloud_file_.is_open()) cloud_file_.close();
    if (pose_socket_ >= 0) close(pose_socket_);
}

void LiDARRecorder::start() {
    running_ = true;
    livox_->initialize();
    
    recording_thread_ = std::thread(&LiDARRecorder::recordingLoop, this);
    pose_thread_ = std::thread(&LiDARRecorder::poseListenerLoop, this);
    
    std::cout << "[Recorder] Started - Recording full resolution point clouds" << std::endl;
}

void LiDARRecorder::stop() {
    running_ = false;
    if (recording_thread_.joinable()) recording_thread_.join();
    if (pose_thread_.joinable()) pose_thread_.join();
    livox_->uninitialize();
    
    std::cout << "[Recorder] Stopped. Recorded " << frame_count_ << " frames" << std::endl;
}

void LiDARRecorder::poseListenerLoop() {
    // Simple packet format: timestamp(8) + pos(12) + quat(16) = 36 bytes
    uint8_t buffer[1024];
    
    while (running_) {
        ssize_t n = recv(pose_socket_, buffer, sizeof(buffer), 0);
        
        if (n >= 36) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            
            // Parse packet (assumes little-endian)
            uint64_t* ts = (uint64_t*)buffer;
            float* pos = (float*)(buffer + 8);
            float* quat = (float*)(buffer + 20);
            
            latest_pose_.timestamp_us = *ts;
            latest_pose_.position = Eigen::Vector3f(pos[0], pos[1], pos[2]);
            latest_pose_.orientation = Eigen::Quaternionf(quat[0], quat[1], quat[2], quat[3]);
            latest_pose_.valid = true;
        }
    }
}

void LiDARRecorder::recordingLoop() {
    auto last_log = std::chrono::steady_clock::now();
    
    while (running_) {
        // Get latest point cloud from LiDAR
        auto points = livox_->getLatestPointCloud();
        
        if (points.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        // Get current pose
        PoseData pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pose = latest_pose_;
        }
        
        if (!pose.valid) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        // Write pose to text file
        pose_file_ << pose.timestamp_us << " "
                   << pose.position.x() << " " << pose.position.y() << " " << pose.position.z() << " "
                   << pose.orientation.w() << " " << pose.orientation.x() << " "
                   << pose.orientation.y() << " " << pose.orientation.z() << "\n";
        
        // Write point cloud to binary file
        // Format: frame_header(timestamp + num_points) + points(x,y,z floats)
        cloud_file_.write((char*)&pose.timestamp_us, sizeof(uint64_t));
        uint32_t num_points = points.size();
        cloud_file_.write((char*)&num_points, sizeof(uint32_t));
        
        for (const auto& pt : points) {
            cloud_file_.write((char*)&pt.x(), sizeof(float));
            cloud_file_.write((char*)&pt.y(), sizeof(float));
            cloud_file_.write((char*)&pt.z(), sizeof(float));
        }
        
        frame_count_++;
        
        // Log progress every 5 seconds
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 5) {
            std::cout << "[Recorder] Frames: " << frame_count_ 
                      << ", Last cloud: " << num_points << " points" << std::endl;
            last_log = now;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ~20Hz recording
    }
}
