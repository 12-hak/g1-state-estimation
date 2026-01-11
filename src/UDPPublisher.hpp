#pragma once

#include <Eigen/Dense>
#include <string>
#include <cstdint>
#include <vector>

// Forward declare socket types
struct sockaddr_in;

namespace g1_localization {

#pragma pack(push, 1)
// V4 State Packet
struct StatePacketV4 {
    char magic[4];              // "G1S4"
    double timestamp;
    float position_raw[3];
    float position_icp[3];
    float velocity[2];
    float joint_pos[29];
    float joint_vel[29];
    float imu_quat[4];
    float imu_gyro[3];
    uint32_t point_count;
};

// Map Packet (to view the "First Scan")
struct MapPacket {
    char magic[4];              // "G1M1"
    uint32_t point_count;
    // followed by points (3 floats each)
};
#pragma pack(pop)

class UDPPublisher {
public:
    UDPPublisher(const std::string& receiver_ip, uint16_t port);
    ~UDPPublisher();
    
    void sendState(const Eigen::Vector3f& position_raw,
                   const Eigen::Vector3f& position_icp,
                   const Eigen::Vector2f& velocity,
                   const std::vector<float>& joint_pos,
                   const std::vector<float>& joint_vel,
                   const Eigen::Quaternionf& imu_quat,
                   const Eigen::Vector3f& imu_gyro,
                   const std::vector<Eigen::Vector2f>& point_cloud);

    void sendMap(const std::vector<Eigen::Vector2f>& map_points);
    
    void sendPose(uint64_t timestamp_us,
                  const Eigen::Vector3f& position,
                  const Eigen::Quaternionf& orientation,
                  uint16_t port = 9871);  // Different port for recorder

private:
    int socket_fd_;
    sockaddr_in* receiver_addr_;
};

} // namespace g1_localization
