#include "LivoxInterface.hpp"
#include <iostream>
#include <cstring>
#include <vector>
#include <cmath>
#include <deque>
#include <algorithm>

namespace g1_localization {

LivoxInterface::LivoxInterface()
    : running_(false)
    , imu_orientation_valid_(false)
    , imu_q_(Eigen::Quaternionf::Identity())
    , last_imu_tp_valid_(false)
    , imu_kp_(2.0f)
    , max_dt_s_(0.05f)
    , max_imu_samples_(600)
    , point_time_span_s_(0.01f) {}

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
    SetLivoxLidarImuDataCallback(ImuDataCallback, this);
    
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
    std::vector<TimedPoint> timed;
    timed.reserve(10000);
    {
        std::lock_guard<std::mutex> lock(points_mutex_);
        timed.swap(points_buffer_);
    }

    if (timed.empty()) return {};

    // Copy IMU samples snapshot (avoid holding lock during deskew)
    std::deque<ImuSample> imu_samples;
    Eigen::Quaternionf q_ref = Eigen::Quaternionf::Identity();
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_samples = imu_samples_;
        q_ref = imu_q_;
    }

    auto interpQ = [&](const std::chrono::steady_clock::time_point& tp) -> Eigen::Quaternionf {
        if (imu_samples.empty()) return q_ref;
        if (tp <= imu_samples.front().tp) return imu_samples.front().q;
        if (tp >= imu_samples.back().tp) return imu_samples.back().q;

        // Linear scan is fine (buffer is small). If needed we can binary-search later.
        for (size_t i = 1; i < imu_samples.size(); ++i) {
            if (tp <= imu_samples[i].tp) {
                const auto& a = imu_samples[i - 1];
                const auto& b = imu_samples[i];
                const float denom = std::chrono::duration_cast<std::chrono::duration<float>>(b.tp - a.tp).count();
                float t = 0.0f;
                if (denom > 1e-6f) {
                    t = std::chrono::duration_cast<std::chrono::duration<float>>(tp - a.tp).count() / denom;
                    t = std::clamp(t, 0.0f, 1.0f);
                }
                return a.q.slerp(t, b.q).normalized();
            }
        }
        return imu_samples.back().q;
    };

    std::vector<Eigen::Vector3f> out;
    out.reserve(timed.size());

    // Deskew: rotate each point into the lidar frame at tp_ref.
    // q is IMU->world; so lidar(frame at tp_ref) <- lidar(frame at tp_point) is q_ref^{-1} * q_point.
    for (const auto& tp : timed) {
        const Eigen::Quaternionf q_point = interpQ(tp.tp);
        const Eigen::Quaternionf q_delta = (q_ref.conjugate() * q_point).normalized();
        out.emplace_back(q_delta * tp.p);
    }

    return out;
}

LidarImu LivoxInterface::getLatestImu() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return latest_imu_;
}

Eigen::Quaternionf LivoxInterface::getLatestImuOrientation() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return imu_q_;
}

bool LivoxInterface::hasImuOrientation() const {
    return imu_orientation_valid_.load();
}

void LivoxInterface::processPoint(float x, float y, float z, const std::chrono::steady_clock::time_point& tp) {
    std::lock_guard<std::mutex> lock(points_mutex_);
    if (points_buffer_.size() >= 10000) return;

    points_buffer_.push_back(TimedPoint{
        Eigen::Vector3f(x / 1000.0f, -y / 1000.0f, -z / 1000.0f),
        tp
    });
}

void LivoxInterface::updateOrientationFromImuLocked(float gyro_x, float gyro_y, float gyro_z,
                                                    float acc_x, float acc_y, float acc_z,
                                                    const std::chrono::steady_clock::time_point& tp) {
    // latest_imu_ and imu_q_ are protected by imu_mutex_ at callsite.
    float dt = 0.01f;
    if (last_imu_tp_valid_) {
        dt = std::chrono::duration_cast<std::chrono::duration<float>>(tp - last_imu_tp_).count();
        if (dt <= 0.0f) dt = 0.001f;
        if (dt > max_dt_s_) dt = max_dt_s_;
    }
    last_imu_tp_ = tp;
    last_imu_tp_valid_ = true;

    // Normalize accelerometer (in g), use only if magnitude near 1g.
    const float an = std::sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
    bool accel_ok = (an > 0.6f && an < 1.4f);

    // Predicted gravity direction in IMU frame, assuming world gravity is (0,0,-1).
    // q_ maps IMU->world, so world->IMU is q_.conjugate().
    Eigen::Vector3f g_pred_imu = (imu_q_.conjugate() * Eigen::Vector3f(0.f, 0.f, -1.f)).normalized();
    Eigen::Vector3f a_meas_imu = accel_ok ? Eigen::Vector3f(acc_x, acc_y, acc_z).normalized()
                                          : Eigen::Vector3f::Zero();

    Eigen::Vector3f omega(gyro_x, gyro_y, gyro_z);
    if (accel_ok) {
        // Error is axis to rotate predicted gravity toward measured accel.
        Eigen::Vector3f err = g_pred_imu.cross(a_meas_imu);
        omega += imu_kp_ * err;
    }

    // Integrate quaternion: q_dot = 0.5 * q ⊗ [0, omega]
    Eigen::Quaternionf dq;
    const float half_dt = 0.5f * dt;
    dq.w() = 1.0f;
    dq.x() = omega.x() * half_dt;
    dq.y() = omega.y() * half_dt;
    dq.z() = omega.z() * half_dt;
    dq.normalize();

    imu_q_ = (imu_q_ * dq).normalized();
    imu_orientation_valid_.store(true);

    imu_samples_.push_back(ImuSample{tp, imu_q_});
    while (imu_samples_.size() > max_imu_samples_) imu_samples_.pop_front();
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

    const auto tp_end = std::chrono::steady_clock::now();
    const int n = static_cast<int>(data->dot_num);
    if (n <= 0) return;
    const float span_s = std::clamp(self->point_time_span_s_, 0.001f, 0.05f);
    const auto tp_start = tp_end - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<float>(span_s));

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        LivoxLidarCartesianHighRawPoint *points = (LivoxLidarCartesianHighRawPoint *)data->data;
        for (int i = 0; i < n; i++) {
            if (points[i].x == 0 && points[i].y == 0 && points[i].z == 0) continue;
            const float alpha = (n > 1) ? (static_cast<float>(i) / static_cast<float>(n - 1)) : 1.0f;
            const auto tp_i = tp_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                          std::chrono::duration<float>(alpha * span_s));
            self->processPoint(points[i].x, points[i].y, points[i].z, tp_i);
        }
    } else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
        LivoxLidarCartesianLowRawPoint *points = (LivoxLidarCartesianLowRawPoint *)data->data;
        for (int i = 0; i < n; i++) {
            if (points[i].x == 0 && points[i].y == 0 && points[i].z == 0) continue;
            const float alpha = (n > 1) ? (static_cast<float>(i) / static_cast<float>(n - 1)) : 1.0f;
            const auto tp_i = tp_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                          std::chrono::duration<float>(alpha * span_s));
            self->processPoint(points[i].x, points[i].y, points[i].z, tp_i);
        }
    }
}

void LivoxInterface::ImuDataCallback(uint32_t handle, const uint8_t dev_type,
                                     LivoxLidarEthernetPacket* data, void* client_data) {
    LivoxInterface* self = static_cast<LivoxInterface*>(client_data);
    if (!self || !data) return;

    if (data->data_type == kLivoxLidarImuData) {
        LivoxLidarImuRawPoint *imu_data = (LivoxLidarImuRawPoint *)data->data;
        const auto tp = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(self->imu_mutex_);
        // Match IMU axes to our point cloud axes transform in processPoint: (x, -y, -z).
        // Apply the same sign flips to gyro/accel so leveling is consistent with point coordinates.
        const float gx = imu_data->gyro_x;
        const float gy = -imu_data->gyro_y;
        const float gz = -imu_data->gyro_z;
        const float ax = imu_data->acc_x;
        const float ay = -imu_data->acc_y;
        const float az = -imu_data->acc_z;

        self->latest_imu_.gyro[0] = gx;
        self->latest_imu_.gyro[1] = gy;
        self->latest_imu_.gyro[2] = gz;
        self->latest_imu_.accel[0] = ax;
        self->latest_imu_.accel[1] = ay;
        self->latest_imu_.accel[2] = az;

        self->updateOrientationFromImuLocked(gx, gy, gz, ax, ay, az, tp);
    }
}

} // namespace g1_localization
