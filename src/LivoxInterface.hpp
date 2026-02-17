#pragma once

#include <Eigen/Dense>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdint>
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

namespace g1_localization {

class LivoxInterface {
public:
    LivoxInterface();
    ~LivoxInterface();
    
    void initialize();
    void uninitialize();
    
    std::vector<Eigen::Vector3f> getLatestPointCloud();
    
    // SDK Callbacks (must be static)
    static void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
    static void WorkModeCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data);
    static void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);

private:
    void processPoint(float x, float y, float z);

    std::atomic<bool> running_;
    
    std::vector<Eigen::Vector3f> points_buffer_;
    std::mutex points_mutex_;
};

} // namespace g1_localization
