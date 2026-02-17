#include "LivoxInterface.hpp"
#include <iostream>
#include <cstring>
#include <vector>

namespace g1_localization {

LivoxInterface::LivoxInterface() : running_(false) {}

LivoxInterface::~LivoxInterface() {
    uninitialize();
}

void LivoxInterface::initialize() {
    if (running_) return;
    
    // Try absolute path first (Robot Desktop path), then local
    const char* config_paths[] = {
        "/home/unitree/development/state_e/g1_mid360_config.json",
        "g1_mid360_config.json",
        "./g1_mid360_config.json"
    };

    bool inited = false;
    for (const char* path : config_paths) {
        if (LivoxLidarSdkInit(path)) {
            std::cout << "[LivoxInterface] SDK Initialized using: " << path << std::endl;
            inited = true;
            break;
        }
    }

    if (!inited) {
        std::cerr << "[LivoxInterface] ERROR: Livox SDK Init Failed. Config NOT found!" << std::endl;
        return;
    }
    
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, this);
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);
    SetLivoxLidarImuDataCallBack(ImuDataCallback, this);
    
    running_ = true;
    std::cout << "[LivoxInterface] SDK Initialized." << std::endl;
}

void LivoxInterface::uninitialize() {
    if (!running_) return;
    running_ = false;
    LivoxLidarSdkUninit();
    std::cout << "[LivoxInterface] Uninitialized" << std::endl;
}

std::vector<Eigen::Vector3f> LivoxInterface::getLatestPointCloud() {
    std::lock_guard<std::mutex> lock(points_mutex_);
    auto points = points_buffer_;
    points_buffer_.clear();
    return points;
}

LivoxInterface::LidarImu LivoxInterface::getLatestImu() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return latest_imu_;
}

void LivoxInterface::processPoint(float x, float y, float z) {
    std::lock_guard<std::mutex> lock(points_mutex_);
    if (points_buffer_.size() >= 10000) return; 
    
    // Convert mm to meters and apply basic transform
    points_buffer_.emplace_back(x / 1000.0f, -y / 1000.0f, -z / 1000.0f);
}

// ---------------- STATIC CALLBACKS ---------------- //

void LivoxInterface::WorkModeCallback(livox_status status, uint32_t handle,
                                      LivoxLidarAsyncControlResponse *response, void *client_data) {
    if (response != nullptr && response->ret_code == 0) {
        std::cout << "[LivoxInterface] LiDAR streaming started" << std::endl;
    }
}

void LivoxInterface::LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
    if (info == nullptr) return;
    std::cout << "[LivoxInterface] Connected: " << info->sn << std::endl;
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
}

void LivoxInterface::PointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                        LivoxLidarEthernetPacket* data, void* client_data) {
    LivoxInterface* self = static_cast<LivoxInterface*>(client_data);
    if (!self || !data) return;

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        LivoxLidarCartesianHighRawPoint *points = (LivoxLidarCartesianHighRawPoint *)data->data;
        for (int i = 0; i < data->dot_num; i++) {
            if (points[i].x == 0 && points[i].y == 0 && points[i].z == 0) continue;
            self->processPoint(points[i].x, points[i].y, points[i].z);
        }
    } else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
        LivoxLidarCartesianLowRawPoint *points = (LivoxLidarCartesianLowRawPoint *)data->data;
        for (int i = 0; i < data->dot_num; i++) {
            if (points[i].x == 0 && points[i].y == 0 && points[i].z == 0) continue;
            self->processPoint(points[i].x, points[i].y, points[i].z);
        }
    }
}

void LivoxInterface::ImuDataCallback(uint32_t handle, const uint8_t dev_type,
                                     LivoxLidarEthernetPacket* data, void* client_data) {
    LivoxInterface* self = static_cast<LivoxInterface*>(client_data);
    if (!self || !data) return;

    if (data->data_type == kLivoxLidarImuData) {
        LivoxLidarImuRawPoint *imu_data = (LivoxLidarImuRawPoint *)data->data;
        std::lock_guard<std::mutex> lock(self->imu_mutex_);
        self->latest_imu_.gyro[0] = imu_data->gyro_x;
        self->latest_imu_.gyro[1] = imu_data->gyro_y;
        self->latest_imu_.gyro[2] = imu_data->gyro_z;
        self->latest_imu_.accel[0] = imu_data->acc_x;
        self->latest_imu_.accel[1] = imu_data->acc_y;
        self->latest_imu_.accel[2] = imu_data->acc_z;
    }
}

} // namespace g1_localization
