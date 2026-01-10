#pragma once

#include <Eigen/Dense>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

namespace g1_localization {

class LivoxInterface {
public:
    LivoxInterface();
    ~LivoxInterface();
    
    void initialize();
    void uninitialize();
    
    std::vector<Eigen::Vector3f> getLatestPointCloud();
    
private:
    void receiveLoop();
    
    std::atomic<bool> running_;
    int socket_fd_;
    uint16_t port_;
    
    std::vector<Eigen::Vector3f> points_buffer_;
    std::mutex points_mutex_;
    
    std::thread receiver_thread_;
};

} // namespace g1_localization
