#include "LivoxInterface.hpp"
#include <iostream>
#include <cstring>

namespace g1_localization {

LivoxInterface::LivoxInterface() 
    : running_(false)
    , socket_fd_(-1)
    , port_(56300) {
}

LivoxInterface::~LivoxInterface() {
    uninitialize();
}

void LivoxInterface::initialize() {
    if (running_) return;
    
    // Create UDP socket for Livox data
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "[LivoxInterface] Failed to create socket" << std::endl;
        return;
    }
    
    // Bind to Livox port
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port_);
    
    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[LivoxInterface] Failed to bind to port " << port_ << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return;
    }
    
    // Set socket timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    running_ = true;
    receiver_thread_ = std::thread(&LivoxInterface::receiveLoop, this);
    
    std::cout << "[LivoxInterface] Initialized on port " << port_ << std::endl;
}

void LivoxInterface::uninitialize() {
    if (!running_) return;
    
    running_ = false;
    
    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }
    
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    
    std::cout << "[LivoxInterface] Uninitialized" << std::endl;
}

std::vector<Eigen::Vector3f> LivoxInterface::getLatestPointCloud() {
    std::lock_guard<std::mutex> lock(points_mutex_);
    auto points = points_buffer_;
    points_buffer_.clear();
    return points;
}

void LivoxInterface::receiveLoop() {
    uint8_t buffer[2048];
    
    while (running_) {
        ssize_t received = recv(socket_fd_, buffer, sizeof(buffer), 0);
        
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;  // Timeout, try again
            }
            break;
        }
        
        if (received < 24) continue;  // Too small for Livox packet
        
        // Parse Livox packet
        // Header: version(1) + slot(1) + id(1) + reserved(1) + err_code(4) + 
        //         timestamp_type(1) + data_type(1) + timestamp(8) + length(4)
        
        uint8_t data_type = buffer[9];
        uint32_t length = *reinterpret_cast<uint32_t*>(&buffer[18]);
        
        if (received < 24 + length) continue;
        
        std::vector<Eigen::Vector3f> new_points;
        
        if (data_type == 0) {
            // Cartesian coordinate format (x, y, z, reflectivity)
            int num_points = length / 13;
            for (int i = 0; i < num_points; ++i) {
                int offset = 24 + i * 13;
                int32_t x = *reinterpret_cast<int32_t*>(&buffer[offset]);
                int32_t y = *reinterpret_cast<int32_t*>(&buffer[offset + 4]);
                int32_t z = *reinterpret_cast<int32_t*>(&buffer[offset + 8]);
                
                // Convert from mm to meters
                // UPSIDE DOWN MOUNT: X is forward, Y is flipped, Z is flipped
                new_points.emplace_back(
                    x / 1000.0f,
                   -y / 1000.0f, // Flip Y: Left becomes Right
                   -z / 1000.0f  // Flip Z: Up becomes Down
                );
            }
        } else if (data_type == 1) {
            // High performance format (x, y, z, reflectivity, tag)
            int num_points = length / 14;
            for (int i = 0; i < num_points; ++i) {
                int offset = 24 + i * 14;
                int32_t x = *reinterpret_cast<int32_t*>(&buffer[offset]);
                int32_t y = *reinterpret_cast<int32_t*>(&buffer[offset + 4]);
                int32_t z = *reinterpret_cast<int32_t*>(&buffer[offset + 8]);
                
                new_points.emplace_back(
                    x / 1000.0f,
                   -y / 1000.0f, // Flip Y
                   -z / 1000.0f  // Flip Z
                );
            }
        }
        
        if (!new_points.empty()) {
            std::lock_guard<std::mutex> lock(points_mutex_);
            points_buffer_.insert(points_buffer_.end(), new_points.begin(), new_points.end());
            
            // Keep buffer size manageable
            if (points_buffer_.size() > 5000) {
                points_buffer_.erase(points_buffer_.begin(), 
                                    points_buffer_.begin() + (points_buffer_.size() - 5000));
            }
        }
    }
}

} // namespace g1_localization
