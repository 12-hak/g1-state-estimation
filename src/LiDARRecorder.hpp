#pragma once

#include "LivoxInterface.hpp"
#include <fstream>
#include <vector>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <Eigen/Dense>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace g1_localization {

struct PoseData {
    uint64_t timestamp_us;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    bool valid;
};

class LiDARRecorder {
public:
    LiDARRecorder(const std::string& output_dir = ".",
                  const std::string& localizer_ip = "127.0.0.1",
                  uint16_t localizer_port = 9871);
    ~LiDARRecorder();
    
    void start();
    void stop();
    
private:
    void recordingLoop();
    void poseListenerLoop();
    
    std::unique_ptr<LivoxInterface> livox_;
    std::string output_dir_;
    
    // Pose from localizer
    PoseData latest_pose_;
    std::mutex pose_mutex_;
    int pose_socket_;
    
    // Recording control
    std::atomic<bool> running_;
    std::thread recording_thread_;
    std::thread pose_thread_;
    
    // Output files
    std::ofstream pose_file_;
    std::ofstream cloud_file_;
    uint64_t frame_count_;
};

} // namespace g1_localization
