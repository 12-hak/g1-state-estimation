#pragma once

#include <Eigen/Dense>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <deque>
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

namespace g1_localization {

    struct LidarImu {
        float gyro[3]; // rad/s
        float accel[3]; // g
        uint64_t timestamp;
    };

class LivoxInterface {
public:
    LivoxInterface();
    ~LivoxInterface();
    
    void initialize();
    void uninitialize();
    

    std::vector<Eigen::Vector3f> getLatestPointCloud();
    LidarImu getLatestImu();
    Eigen::Quaternionf getLatestImuOrientation();
    bool hasImuOrientation() const;
    
    // SDK Callbacks (must be static)
    static void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
    static void ImuDataCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
    static void WorkModeCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data);
    static void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);

private:
    struct TimedPoint {
        Eigen::Vector3f p;
        std::chrono::steady_clock::time_point tp;
    };

    struct ImuSample {
        std::chrono::steady_clock::time_point tp;
        Eigen::Quaternionf q;
    };

    void processPoint(float x, float y, float z, const std::chrono::steady_clock::time_point& tp);
    void updateOrientationFromImuLocked(float gyro_x, float gyro_y, float gyro_z,
                                        float acc_x, float acc_y, float acc_z,
                                        const std::chrono::steady_clock::time_point& tp);

    std::atomic<bool> running_;
    
    std::vector<TimedPoint> points_buffer_;
    std::mutex points_mutex_;

    LidarImu latest_imu_;
    std::mutex imu_mutex_;

    // Simple IMU fusion for roll/pitch leveling (Mahony-style), yaw is gyro-integrated only.
    std::atomic<bool> imu_orientation_valid_;
    Eigen::Quaternionf imu_q_;
    std::chrono::steady_clock::time_point last_imu_tp_;
    bool last_imu_tp_valid_;
    float imu_kp_;
    float max_dt_s_;

    // Buffer of orientations for deskewing recent points
    std::deque<ImuSample> imu_samples_;
    size_t max_imu_samples_;

    // Approximate capture time span for a packet's points (seconds)
    float point_time_span_s_;
};

} // namespace g1_localization
