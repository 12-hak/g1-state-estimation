#include "g1_sensor_bridge/livox_bridge.hpp"
#include <sensor_msgs/msg/point_field.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <cmath>

namespace g1_sensor_bridge {

LivoxBridgeNode::LivoxBridgeNode(const rclcpp::NodeOptions& options)
    : Node("livox_bridge_node", options) {

    declare_parameter("frame_id", "lidar_link");
    declare_parameter("lidar_config_path", "");
    declare_parameter("body_filter_radius", 0.3);
    declare_parameter("max_range", 15.0);
    declare_parameter("min_range", 0.15);
    declare_parameter("flip_lidar", true);
    declare_parameter("lidar_offset_x", 0.10);
    declare_parameter("lidar_offset_y", 0.0);
    declare_parameter("lidar_offset_z", 0.60);
    declare_parameter("publish_rate", 10.0);
    declare_parameter("use_stabilized", true);
    declare_parameter("deskew_span_s", 0.012);

    frame_id_ = get_parameter("frame_id").as_string();
    lidar_config_path_ = get_parameter("lidar_config_path").as_string();
    body_filter_radius_ = get_parameter("body_filter_radius").as_double();
    max_range_ = get_parameter("max_range").as_double();
    min_range_ = get_parameter("min_range").as_double();
    flip_lidar_ = get_parameter("flip_lidar").as_bool();
    lidar_offset_ = Eigen::Vector3f(
        get_parameter("lidar_offset_x").as_double(),
        get_parameter("lidar_offset_y").as_double(),
        get_parameter("lidar_offset_z").as_double()
    );
    use_stabilized_ = get_parameter("use_stabilized").as_bool();
    deskew_span_s_ = static_cast<float>(std::max(0.001, std::min(0.05, get_parameter("deskew_span_s").as_double())));
    max_imu_samples_ = 600;
    imu_q_ = Eigen::Quaternionf::Identity();
    last_imu_tp_valid_ = false;
    imu_kp_ = 2.0f;
    imu_max_dt_s_ = 0.05f;

    pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/lidar", 50);

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishStaticTransforms();

    double rate = get_parameter("publish_rate").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&LivoxBridgeNode::publishTimerCallback, this));

    initializeLivox();
    RCLCPP_INFO(get_logger(), "Livox bridge initialized (flip=%s, range=%.1f-%.1fm, stabilized=%s)",
                flip_lidar_ ? "true" : "false", min_range_, max_range_,
                use_stabilized_ ? "true" : "false");
}

LivoxBridgeNode::~LivoxBridgeNode() {
    LivoxLidarSdkUninit();
}

void LivoxBridgeNode::initializeLivox() {
    const char* config = lidar_config_path_.empty() ? nullptr : lidar_config_path_.c_str();
    if (!LivoxLidarSdkInit(config)) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize Livox SDK");
        return;
    }

    SetLivoxLidarPointCloudCallBack(onPointCloud, this);
    SetLivoxLidarImuDataCallback(onImuData, this);
    SetLivoxLidarInfoChangeCallback(onDeviceInfo, this);

    if (!LivoxLidarSdkStart()) {
        RCLCPP_ERROR(get_logger(), "Failed to start Livox SDK");
    }
}

void LivoxBridgeNode::publishStaticTransforms() {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = frame_id_;
    tf.transform.translation.x = lidar_offset_.x();
    tf.transform.translation.y = lidar_offset_.y();
    tf.transform.translation.z = lidar_offset_.z();

    if (flip_lidar_) {
        // 180-degree rotation around X axis for upside-down mount
        tf.transform.rotation.w = 0.0;
        tf.transform.rotation.x = 1.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
    } else {
        tf.transform.rotation.w = 1.0;
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
    }

    tf_static_broadcaster_->sendTransform(tf);
}

void LivoxBridgeNode::onPointCloud(uint32_t, const uint8_t,
                                    LivoxLidarEthernetPacket* data, void* client_data) {
    if (!data || !client_data) return;
    auto* node = static_cast<LivoxBridgeNode*>(client_data);
    const auto packet_time = std::chrono::steady_clock::now();

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        auto* points = reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(data->data);
        node->processPointCloud(points, data->dot_num, packet_time);
    } else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
        // Process low-res points (requires a cast or separate handler, for now we log)
        RCLCPP_DEBUG(node->get_logger(), "Received low-res cartesian data");
    }
}

void LivoxBridgeNode::processPointCloud(const LivoxLidarCartesianHighRawPoint* points,
                                         uint32_t count,
                                         const std::chrono::steady_clock::time_point& packet_time) {
    std::lock_guard<std::mutex> lock(point_mutex_);
    // Accumulate points instead of clearing
    if (point_buffer_.size() > 50000) {
        point_buffer_.erase(point_buffer_.begin(), point_buffer_.begin() + count);
    }
    point_buffer_.reserve(point_buffer_.size() + count);

    const float body_r_sq = body_filter_radius_ * body_filter_radius_;
    const float max_r_sq = max_range_ * max_range_;
    const float min_r_sq = min_range_ * min_range_;

    const float span_s = deskew_span_s_;
    const auto t_end = packet_time;
    const auto t_start = t_end - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<float>(span_s));

    for (uint32_t i = 0; i < count; ++i) {
        float x = points[i].x * 0.001f;  // mm to m
        float y = points[i].y * 0.001f;
        float z = points[i].z * 0.001f;

        if (flip_lidar_) {
            y = -y;
            z = -z;
        }

        float dist_sq = x * x + y * y + z * z;
        if (dist_sq < min_r_sq || dist_sq > max_r_sq) continue;

        float xy_sq = x * x + y * y;
        if (xy_sq < body_r_sq) continue;

        std::chrono::steady_clock::time_point tp = t_end;
        if (use_stabilized_ && count > 1) {
            const float alpha = static_cast<float>(i) / static_cast<float>(count - 1);
            tp = t_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                     std::chrono::duration<float>(alpha * span_s));
        }
        point_buffer_.push_back({Eigen::Vector3f(x, y, z), tp});
    }
}

void LivoxBridgeNode::onImuData(uint32_t, const uint8_t,
                                 LivoxLidarEthernetPacket* data, void* client_data) {
    if (!data || !client_data) return;
    auto* node = static_cast<LivoxBridgeNode*>(client_data);

    // Livox IMU data layout: gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z (float32 each)
    if (data->data_type == kLivoxLidarImuData) {
        float* imu_raw = reinterpret_cast<float*>(data->data);
        // timestamp is uint8_t[8] (little-endian nanoseconds)
        uint64_t ts = 0;
        const uint8_t* t = data->timestamp;
        for (int i = 0; i < 8; i++) ts |= static_cast<uint64_t>(t[i]) << (i * 8);
        node->processImu(imu_raw[0], imu_raw[1], imu_raw[2],
                         imu_raw[3], imu_raw[4], imu_raw[5], ts);
    }
}

void LivoxBridgeNode::processImu(float gx, float gy, float gz,
                                  float ax, float ay, float az, uint64_t ts) {
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (flip_lidar_) {
            latest_imu_.gyro[0] = gx;
            latest_imu_.gyro[1] = -gy;
            latest_imu_.gyro[2] = -gz;
            latest_imu_.accel[0] = ax;
            latest_imu_.accel[1] = -ay;
            latest_imu_.accel[2] = -az;
        } else {
            latest_imu_.gyro[0] = gx;
            latest_imu_.gyro[1] = gy;
            latest_imu_.gyro[2] = gz;
            latest_imu_.accel[0] = ax;
            latest_imu_.accel[1] = ay;
            latest_imu_.accel[2] = az;
        }
        latest_imu_.timestamp = ts;
    }

    const auto tp = std::chrono::steady_clock::now();
    updateOrientationFromImu(
        latest_imu_.gyro[0], latest_imu_.gyro[1], latest_imu_.gyro[2],
        latest_imu_.accel[0], latest_imu_.accel[1], latest_imu_.accel[2], tp);

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.angular_velocity.x = latest_imu_.gyro[0];
    msg.angular_velocity.y = latest_imu_.gyro[1];
    msg.angular_velocity.z = latest_imu_.gyro[2];
    msg.linear_acceleration.x = latest_imu_.accel[0];
    msg.linear_acceleration.y = latest_imu_.accel[1];
    msg.linear_acceleration.z = latest_imu_.accel[2];
    // Orientation not available from LiDAR IMU
    msg.orientation_covariance[0] = -1.0;
    imu_pub_->publish(msg);
}

void LivoxBridgeNode::updateOrientationFromImu(float gx, float gy, float gz,
                                                float ax, float ay, float az,
                                                const std::chrono::steady_clock::time_point& tp) {
    std::lock_guard<std::mutex> lock(imu_orientation_mutex_);
    float dt = 0.01f;
    if (last_imu_tp_valid_) {
        dt = std::chrono::duration_cast<std::chrono::duration<float>>(tp - last_imu_tp_).count();
        if (dt <= 0.f) dt = 0.001f;
        if (dt > imu_max_dt_s_) dt = imu_max_dt_s_;
    }
    last_imu_tp_ = tp;
    last_imu_tp_valid_ = true;

    const float an = std::sqrt(ax*ax + ay*ay + az*az);
    const bool accel_ok = (an > 0.6f && an < 1.4f);
    Eigen::Vector3f g_pred = (imu_q_.conjugate() * Eigen::Vector3f(0.f, 0.f, -1.f)).normalized();
    Eigen::Vector3f a_meas = accel_ok ? Eigen::Vector3f(ax, ay, az).normalized() : Eigen::Vector3f::Zero();
    Eigen::Vector3f omega(gx, gy, gz);
    if (accel_ok) {
        Eigen::Vector3f err = g_pred.cross(a_meas);
        omega += imu_kp_ * err;
    }
    const float half_dt = 0.5f * dt;
    Eigen::Quaternionf dq(1.f, omega.x() * half_dt, omega.y() * half_dt, omega.z() * half_dt);
    dq.normalize();
    imu_q_ = (imu_q_ * dq).normalized();
    imu_samples_.push_back({tp, imu_q_});
    while (imu_samples_.size() > max_imu_samples_) imu_samples_.pop_front();
}

Eigen::Quaternionf LivoxBridgeNode::interpolateOrientation(
    const std::chrono::steady_clock::time_point& tp) const {
    std::lock_guard<std::mutex> lock(imu_orientation_mutex_);
    if (imu_samples_.empty()) return imu_q_;
    if (tp <= imu_samples_.front().tp) return imu_samples_.front().q;
    if (tp >= imu_samples_.back().tp) return imu_samples_.back().q;
    for (size_t i = 1; i < imu_samples_.size(); ++i) {
        if (tp <= imu_samples_[i].tp) {
            const auto& a = imu_samples_[i - 1];
            const auto& b = imu_samples_[i];
            const float denom = std::chrono::duration_cast<std::chrono::duration<float>>(b.tp - a.tp).count();
            float t = 0.f;
            if (denom > 1e-6f)
                t = std::chrono::duration_cast<std::chrono::duration<float>>(tp - a.tp).count() / denom;
            t = std::clamp(t, 0.f, 1.f);
            return a.q.slerp(t, b.q).normalized();
        }
    }
    return imu_samples_.back().q;
}

void LivoxBridgeNode::publishTimerCallback() {
    std::vector<TimedPoint> timed;
    {
        std::lock_guard<std::mutex> lock(point_mutex_);
        if (point_buffer_.empty()) return;
        timed.swap(point_buffer_);
    }

    std::vector<Eigen::Vector3f> points;
    points.reserve(timed.size());
    if (use_stabilized_ && !timed.empty()) {
        const auto q_ref = interpolateOrientation(timed.back().tp);
        for (const auto& pt : timed) {
            const Eigen::Quaternionf q_pt = interpolateOrientation(pt.tp);
            const Eigen::Quaternionf q_delta = (q_ref.conjugate() * q_pt).normalized();
            points.push_back(q_delta * pt.p);
        }
    } else {
        for (const auto& pt : timed)
            points.push_back(pt.p);
    }

    auto msg = sensor_msgs::msg::PointCloud2();
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.height = 1;
    msg.width = points.size();
    msg.is_dense = true;
    msg.is_bigendian = false;
    msg.point_step = 12;  // 3 * float32
    msg.row_step = msg.point_step * msg.width;

    sensor_msgs::msg::PointField fx, fy, fz;
    fx.name = "x"; fx.offset = 0;  fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
    fy.name = "y"; fy.offset = 4;  fy.datatype = sensor_msgs::msg::PointField::FLOAT32; fy.count = 1;
    fz.name = "z"; fz.offset = 8;  fz.datatype = sensor_msgs::msg::PointField::FLOAT32; fz.count = 1;
    msg.fields = {fx, fy, fz};

    msg.data.resize(msg.row_step);
    uint8_t* ptr = msg.data.data();
    for (const auto& p : points) {
        std::memcpy(ptr + 0, &p.x(), 4);
        std::memcpy(ptr + 4, &p.y(), 4);
        std::memcpy(ptr + 8, &p.z(), 4);
        ptr += 12;
    }

    pc_pub_->publish(msg);
}

void LivoxBridgeNode::onDeviceInfo(const uint32_t handle, const LivoxLidarInfo* info, void*) {
    if (info) {
        SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, onWorkModeChange, nullptr);
    }
}

void LivoxBridgeNode::onWorkModeChange(livox_status, uint32_t, LivoxLidarAsyncControlResponse*, void*) {}

}  // namespace g1_sensor_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_sensor_bridge::LivoxBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
