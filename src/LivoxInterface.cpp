#include "LivoxInterface.hpp"
#include <iostream>
#include <cstring>

namespace g1_localization {

LivoxInterface::LivoxInterface() : running_(false) {}

LivoxInterface::~LivoxInterface() {
    uninitialize();
}

void LivoxInterface::initialize() {
    if (running_) return;
    
    // Initialize Livox SDK
    // Assumes config file in current directory
    if (!LivoxLidarSdkInit("g1_mid360_config.json")) {
        std::cerr << "[LivoxInterface] ERROR: Livox SDK Init Failed. Check config file!" << std::endl;
        return;
    }
    
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, this);
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);
    
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

void LivoxInterface::processPoint(float x, float y, float z) {
    std::lock_guard<std::mutex> lock(points_mutex_);
    // Keep buffer manageable
    if (points_buffer_.size() >= 5000) return; 
    
    // Convert mm to meters and apply basic transform (Upside Down Mount)
    points_buffer_.emplace_back(x / 1000.0f, -y / 1000.0f, -z / 1000.0f);
}

// ---------------- STATIC CALLBACKS ---------------- //

void LivoxInterface::WorkModeCallback(livox_status status, uint32_t handle,
                                      LivoxLidarAsyncControlResponse *response, void *client_data) {
    if (response != nullptr && response->ret_code == 0) {
        std::cout << "[LivoxInterface] LiDAR set to NORMAL mode (Streaming)" << std::endl;
    } else {
        std::cerr << "[LivoxInterface] WARNING: Failed to set Work Mode!" << std::endl;
    }
}

void LivoxInterface::LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
    if (info == nullptr) return;
    std::cout << "[LivoxInterface] Connected: " << info->sn << " IP: " << info->lidar_ip << std::endl;
    
    // Auto-Start Streaming
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

} // namespace g1_localization
