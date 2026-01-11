#include "UDPPublisher.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <chrono>
#include <iostream>

namespace g1_localization {

UDPPublisher::UDPPublisher(const std::string& receiver_ip, uint16_t port) {
    receiver_addr_ = new sockaddr_in();
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        delete receiver_addr_;
        throw std::runtime_error("Failed to create UDP socket");
    }
    
    std::memset(receiver_addr_, 0, sizeof(sockaddr_in));
    receiver_addr_->sin_family = AF_INET;
    receiver_addr_->sin_port = htons(port);
    
    if (inet_pton(AF_INET, receiver_ip.c_str(), &receiver_addr_->sin_addr) <= 0) {
        close(socket_fd_);
        delete receiver_addr_;
        throw std::runtime_error("Invalid receiver IP address");
    }
    std::cout << "[UDPPublisher] Initialized to " << receiver_ip << ":" << port << std::endl;
}

UDPPublisher::~UDPPublisher() {
    if (socket_fd_ >= 0) close(socket_fd_);
    delete receiver_addr_;
}

void UDPPublisher::sendState(const Eigen::Vector3f& position_raw,
                              const Eigen::Vector3f& position_icp,
                              const Eigen::Vector2f& velocity,
                              const std::vector<float>& joint_pos,
                              const std::vector<float>& joint_vel,
                              const Eigen::Quaternionf& imu_quat,
                              const Eigen::Vector3f& imu_gyro,
                              const std::vector<Eigen::Vector2f>& point_cloud) {
    StatePacketV4 header;
    std::memcpy(header.magic, "G1S4", 4);
    
    auto now = std::chrono::system_clock::now();
    header.timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
    
    header.position_raw[0] = position_raw.x();
    header.position_raw[1] = position_raw.y();
    header.position_raw[2] = position_raw.z();
    
    header.position_icp[0] = position_icp.x();
    header.position_icp[1] = position_icp.y();
    header.position_icp[2] = position_icp.z();
    
    header.velocity[0] = velocity.x();
    header.velocity[1] = velocity.y();
    
    std::memcpy(header.joint_pos, joint_pos.data(), 29 * sizeof(float));
    std::memcpy(header.joint_vel, joint_vel.data(), 29 * sizeof(float));
    
    header.imu_quat[0] = imu_quat.w();
    header.imu_quat[1] = imu_quat.x();
    header.imu_quat[2] = imu_quat.y();
    header.imu_quat[3] = imu_quat.z();
    
    header.imu_gyro[0] = imu_gyro.x();
    header.imu_gyro[1] = imu_gyro.y();
    header.imu_gyro[2] = imu_gyro.z();
    
    uint32_t point_count = std::min(static_cast<uint32_t>(point_cloud.size()), 100u);
    header.point_count = point_count;
    
    std::vector<float> point_data;
    point_data.reserve(point_count * 3);
    for (uint32_t i = 0; i < point_count; ++i) {
        point_data.push_back(point_cloud[i].x());
        point_data.push_back(point_cloud[i].y());
        point_data.push_back(0.0f);
    }
    
    std::vector<uint8_t> packet;
    packet.resize(sizeof(StatePacketV4) + point_data.size() * sizeof(float));
    std::memcpy(packet.data(), &header, sizeof(StatePacketV4));
    if (!point_data.empty()) {
        std::memcpy(packet.data() + sizeof(StatePacketV4), point_data.data(), point_data.size() * sizeof(float));
    }
    
    sendto(socket_fd_, packet.data(), packet.size(), 0, (struct sockaddr*)receiver_addr_, sizeof(sockaddr_in));
}

void UDPPublisher::sendMap(const std::vector<Eigen::Vector2f>& map_points) {
    if (map_points.empty()) return;
    MapPacket header;
    std::memcpy(header.magic, "G1M1", 4);
    header.point_count = std::min(static_cast<uint32_t>(map_points.size()), 1000u);
    std::vector<float> data;
    data.reserve(header.point_count * 3);
    for (uint32_t i = 0; i < header.point_count; ++i) {
        data.push_back(map_points[i].x());
        data.push_back(map_points[i].y());
        data.push_back(0.0f);
    }
    std::vector<uint8_t> packet;
    packet.resize(sizeof(MapPacket) + data.size() * sizeof(float));
    std::memcpy(packet.data(), &header, sizeof(MapPacket));
    std::memcpy(packet.data() + sizeof(MapPacket), data.data(), data.size() * sizeof(float));
    sendto(socket_fd_, packet.data(), packet.size(), 0, (struct sockaddr*)receiver_addr_, sizeof(sockaddr_in));
}

void UDPPublisher::sendPose(uint64_t timestamp_us,
                             const Eigen::Vector3f& position,
                             const Eigen::Quaternionf& orientation,
                             uint16_t port) {
    // Simple packet: timestamp(8) + pos(12) + quat(16) = 36 bytes
    uint8_t packet[36];
    
    // Timestamp
    std::memcpy(packet, &timestamp_us, sizeof(uint64_t));
    
    // Position
    float pos[3] = {position.x(), position.y(), position.z()};
    std::memcpy(packet + 8, pos, 12);
    
    // Quaternion (w, x, y, z)
    float quat[4] = {orientation.w(), orientation.x(), orientation.y(), orientation.z()};
    std::memcpy(packet + 20, quat, 16);
    
    // Send to recorder port
    sockaddr_in recorder_addr;
    std::memcpy(&recorder_addr, receiver_addr_, sizeof(sockaddr_in));
    recorder_addr.sin_port = htons(port);
    
    sendto(socket_fd_, packet, sizeof(packet), 0, (struct sockaddr*)&recorder_addr, sizeof(sockaddr_in));
}

} // namespace g1_localization
