#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>

#include <livox_lidar_def.h>
#include <livox_lidar_api.h>

#include <mutex>
#include <atomic>
#include <vector>
#include <string>

namespace g1_sensor_bridge {

class LivoxBridgeNode : public rclcpp::Node {
public:
    explicit LivoxBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LivoxBridgeNode() override;

private:
    void initializeLivox();
    void publishTimerCallback();
    void publishStaticTransforms();

    static void onPointCloud(uint32_t handle, const uint8_t dev_type,
                             LivoxLidarEthernetPacket* data, void* client_data);
    static void onImuData(uint32_t handle, const uint8_t dev_type,
                          LivoxLidarEthernetPacket* data, void* client_data);
    static void onDeviceInfo(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);
    static void onWorkModeChange(livox_status status, uint32_t handle,
                                 LivoxLidarAsyncControlResponse* response, void* client_data);

    void processPointCloud(const LivoxLidarCartesianHighRawPoint* points, uint32_t count);
    void processImu(float gyro_x, float gyro_y, float gyro_z,
                    float acc_x, float acc_y, float acc_z, uint64_t timestamp);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    std::vector<Eigen::Vector3f> point_buffer_;
    std::mutex point_mutex_;

    struct ImuData {
        float gyro[3] = {0};
        float accel[3] = {0};
        uint64_t timestamp = 0;
    };
    ImuData latest_imu_;
    std::mutex imu_mutex_;

    // Parameters
    std::string frame_id_;
    std::string lidar_config_path_;
    double body_filter_radius_;
    double max_range_;
    double min_range_;
    bool flip_lidar_;  // Livox Mid-360 on G1 is upside-down
    Eigen::Vector3f lidar_offset_;
};

}  // namespace g1_sensor_bridge
