#include "g1_sensor_bridge/livox_bridge.hpp"
#include <sensor_msgs/msg/point_field.hpp>
#include <chrono>
#include <cstring>

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
    RCLCPP_INFO(get_logger(), "Livox bridge initialized (flip=%s, range=%.1f-%.1fm)",
                flip_lidar_ ? "true" : "false", min_range_, max_range_);
}

LivoxBridgeNode::~LivoxBridgeNode() {
    LivoxLidarSdkUninit();
}

void LivoxBridgeNode::initializeLivox() {
    const char* config = lidar_config_path_.empty() ? nullptr : lidar_config_path_.c_str();
    if (LivoxLidarSdkInit(config) != kLivoxLidarStatusSuccess) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize Livox SDK");
        return;
    }

    SetLivoxLidarPointCloudCallBack(onPointCloud, this);
    SetLivoxLidarImuDataCallback(onImuData, this);
    SetLivoxLidarInfoChangeCallback(onDeviceInfo, this);

    if (LivoxLidarSdkStart() != kLivoxLidarStatusSuccess) {
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

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        auto* points = reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(data->data);
        node->processPointCloud(points, data->dot_num);
    }
}

void LivoxBridgeNode::processPointCloud(const LivoxLidarCartesianHighRawPoint* points,
                                         uint32_t count) {
    std::lock_guard<std::mutex> lock(point_mutex_);
    point_buffer_.clear();
    point_buffer_.reserve(count);

    const float body_r_sq = body_filter_radius_ * body_filter_radius_;
    const float max_r_sq = max_range_ * max_range_;
    const float min_r_sq = min_range_ * min_range_;

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

        point_buffer_.emplace_back(x, y, z);
    }
}

void LivoxBridgeNode::onImuData(uint32_t, const uint8_t,
                                 LivoxLidarEthernetPacket* data, void* client_data) {
    if (!data || !client_data) return;
    auto* node = static_cast<LivoxBridgeNode*>(client_data);

    // Livox IMU data layout: gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z (float32 each)
    if (data->data_type == kLivoxLidarImuData) {
        float* imu_raw = reinterpret_cast<float*>(data->data);
        node->processImu(imu_raw[0], imu_raw[1], imu_raw[2],
                         imu_raw[3], imu_raw[4], imu_raw[5],
                         data->time_stamp);
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

void LivoxBridgeNode::publishTimerCallback() {
    std::vector<Eigen::Vector3f> points;
    {
        std::lock_guard<std::mutex> lock(point_mutex_);
        if (point_buffer_.empty()) return;
        points.swap(point_buffer_);
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
    std::memcpy(msg.data.data(), points.data(), msg.row_step);

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
