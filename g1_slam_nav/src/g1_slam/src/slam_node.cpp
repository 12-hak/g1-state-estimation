#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "g1_slam/kiss_icp_wrapper.hpp"
#include "g1_slam/ekf_fusion.hpp"
#include "g1_slam/loop_closure.hpp"
#include "g1_slam/pose_graph.hpp"
#include "g1_msgs/msg/slam_status.hpp"

#include <mutex>
#include <chrono>
#include <algorithm>
#include <cmath>

namespace g1_slam {

class SlamNode : public rclcpp::Node {
public:
    explicit SlamNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("slam_node", options) {

        declareParameters();
        initComponents();

        pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", rclcpp::SensorDataQoS(),
            std::bind(&SlamNode::pointCloudCallback, this, std::placeholders::_1));

        leg_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom/legs", 50,
            std::bind(&SlamNode::legOdomCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu/body", 50,
            std::bind(&SlamNode::imuCallback, this, std::placeholders::_1));

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("slam/pose", 10);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("slam/odom", 10);
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("slam/map", 1);
        status_pub_ = create_publisher<g1_msgs::msg::SlamStatus>("slam/status", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        auto map_period = std::chrono::duration<double>(1.0 / map_publish_rate_);
        map_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(map_period),
            std::bind(&SlamNode::publishMap, this));

        RCLCPP_INFO(get_logger(), "SLAM node initialized (voxel=%.2f, range=%.1f-%.1f)",
                    icp_config_.voxel_size, icp_config_.min_range, icp_config_.max_range);
    }

private:
    void declareParameters() {
        declare_parameter("voxel_size", 0.5);
        declare_parameter("max_range", 15.0);
        declare_parameter("min_range", 0.3);
        declare_parameter("max_icp_iterations", 50);
        declare_parameter("odom_frame", "odom");
        declare_parameter("base_frame", "base_link");
        declare_parameter("map_frame", "map");
        declare_parameter("map_publish_rate", 1.0);
        declare_parameter("keyframe_distance", 1.0);
        declare_parameter("keyframe_angle", 0.5);
        declare_parameter("loop_search_radius", 15.0);
        declare_parameter("enable_loop_closure", true);

        icp_config_.voxel_size = get_parameter("voxel_size").as_double();
        icp_config_.max_range = get_parameter("max_range").as_double();
        icp_config_.min_range = get_parameter("min_range").as_double();
        icp_config_.max_iterations = get_parameter("max_icp_iterations").as_int();
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        map_frame_ = get_parameter("map_frame").as_string();
        map_publish_rate_ = get_parameter("map_publish_rate").as_double();
        enable_loop_closure_ = get_parameter("enable_loop_closure").as_bool();

        lc_config_.keyframe_distance = get_parameter("keyframe_distance").as_double();
        lc_config_.keyframe_angle = get_parameter("keyframe_angle").as_double();
        lc_config_.search_radius = get_parameter("loop_search_radius").as_double();
    }

    void initComponents() {
        kiss_icp_ = std::make_unique<KissICPWrapper>(icp_config_);
        ekf_ = std::make_unique<EKFFusion>();
        loop_detector_ = std::make_unique<LoopClosureDetector>(lc_config_);
        pose_graph_ = std::make_unique<PoseGraph>();
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        std::vector<Eigen::Vector3d> points;
        points.reserve(pcl_cloud.size());
        for (const auto& p : pcl_cloud) {
            if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                points.emplace_back(p.x, p.y, p.z);
            }
        }

        if (points.empty()) return;

        // Get initial guess from EKF
        Eigen::Matrix4d guess = ekf_->getPose();

        // Register frame with KISS-ICP
        auto result = kiss_icp_->registerFrame(points, guess);

        if (!result.converged) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "ICP did not converge (fitness=%.4f)", result.fitness_score);
            return;
        }

        // Update EKF with LiDAR measurement
        Eigen::Matrix<double, 6, 6> lidar_cov = Eigen::Matrix<double, 6, 6>::Identity();
        lidar_cov *= 0.01;
        lidar_cov(2,2) = 0.05;  // z less certain
        ekf_->updateLidar(result.pose, lidar_cov);

        // Get fused pose
        Eigen::Matrix4d fused = ekf_->getPose();
        auto stamp = msg->header.stamp;

        // Publish pose
        publishPose(fused, stamp);
        publishOdometry(fused, stamp);
        publishTF(fused, stamp);

        // Loop closure
        if (enable_loop_closure_) {
            double ts = stamp.sec + stamp.nanosec * 1e-9;
            if (loop_detector_->addKeyframe(fused, points, ts)) {
                // Add odometry edge to pose graph
                int kf_id = loop_detector_->numKeyframes() - 1;
                pose_graph_->addNode(kf_id, fused);

                if (kf_id > 0) {
                    Eigen::Matrix4d prev = pose_graph_->getNodePose(kf_id - 1);
                    Eigen::Matrix4d rel = prev.inverse() * fused;
                    Eigen::Matrix<double, 6, 6> odom_info = Eigen::Matrix<double, 6, 6>::Identity() * 100.0;
                    pose_graph_->addEdge(kf_id - 1, kf_id, rel, odom_info);
                }

                auto loop = loop_detector_->detectLoop();
                if (loop.has_value()) {
                    RCLCPP_INFO(get_logger(), "Loop closure detected: %d -> %d (score=%.4f)",
                                loop->from_id, loop->to_id, loop->score);
                    pose_graph_->addEdge(loop->from_id, loop->to_id,
                                          loop->relative_pose, loop->information);

                    if (pose_graph_->optimize(10)) {
                        RCLCPP_INFO(get_logger(), "Pose graph optimized (%zu nodes, %zu edges)",
                                    pose_graph_->numNodes(), pose_graph_->numEdges());
                        loop_closure_count_++;
                    }
                }
            }
        }

        // Update status
        auto status = g1_msgs::msg::SlamStatus();
        status.header.stamp = stamp;
        status.mapping_active = true;
        status.localization_valid = result.converged;
        status.map_point_count = static_cast<uint32_t>(kiss_icp_->localMap().size());
        status.loop_closure_count = loop_closure_count_;
        status.odometry_quality = 1.0 - std::min(result.fitness_score, 1.0);
        status.map_resolution = icp_config_.voxel_size;
        status_pub_->publish(status);
    }

    void legOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double current_ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        double dt = (last_leg_odom_ts_ > 0) ? (current_ts - last_leg_odom_ts_) : 0.02;
        last_leg_odom_ts_ = current_ts;
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        // Predict first, then update (standard EKF cycle)
        Eigen::Vector3d lin_vel(msg->twist.twist.linear.x,
                                 msg->twist.twist.linear.y,
                                 msg->twist.twist.linear.z);
        Eigen::Vector3d ang_vel(msg->twist.twist.angular.x,
                                 msg->twist.twist.angular.y,
                                 msg->twist.twist.angular.z);
        ekf_->predict(lin_vel, ang_vel, dt);

        Eigen::Vector3d pos(msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z);
        Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                              msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z);

        Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();
        cov(0,0) = msg->pose.covariance[0];
        cov(1,1) = msg->pose.covariance[7];
        cov(2,2) = msg->pose.covariance[14];
        cov(3,3) = msg->pose.covariance[21];
        cov(4,4) = msg->pose.covariance[28];
        cov(5,5) = msg->pose.covariance[35];

        ekf_->updateLegOdom(pos, q, cov);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double current_ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        double dt = (last_imu_ts_ > 0) ? (current_ts - last_imu_ts_) : 0.02;
        last_imu_ts_ = current_ts;
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        Eigen::Vector3d ang_vel(msg->angular_velocity.x,
                                 msg->angular_velocity.y,
                                 msg->angular_velocity.z);
        ekf_->predict(Eigen::Vector3d::Zero(), ang_vel, dt);
    }

    void publishPose(const Eigen::Matrix4d& pose, const rclcpp::Time& stamp) {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = stamp;
        msg.header.frame_id = map_frame_;

        msg.pose.position.x = pose(0,3);
        msg.pose.position.y = pose(1,3);
        msg.pose.position.z = pose(2,3);

        Eigen::Quaterniond q(pose.block<3,3>(0,0));
        msg.pose.orientation.w = q.w();
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();

        pose_pub_->publish(msg);
    }

    void publishOdometry(const Eigen::Matrix4d& pose, const rclcpp::Time& stamp) {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = stamp;
        msg.header.frame_id = map_frame_;
        msg.child_frame_id = base_frame_;

        msg.pose.pose.position.x = pose(0,3);
        msg.pose.pose.position.y = pose(1,3);
        msg.pose.pose.position.z = pose(2,3);

        Eigen::Quaterniond q(pose.block<3,3>(0,0));
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        // Set covariance from EKF
        auto cov = ekf_->getCovariance();
        for (int i = 0; i < 6; ++i) {
            msg.pose.covariance[i * 7] = cov(i, i);
        }

        odom_pub_->publish(msg);
    }

    void publishTF(const Eigen::Matrix4d& pose, const rclcpp::Time& stamp) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = map_frame_;
        tf.child_frame_id = odom_frame_;

        // map -> odom transform (corrects odom drift)
        tf.transform.translation.x = pose(0,3);
        tf.transform.translation.y = pose(1,3);
        tf.transform.translation.z = pose(2,3);

        Eigen::Quaterniond q(pose.block<3,3>(0,0));
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(tf);
    }

    void publishMap() {
        auto points = kiss_icp_->getMapPoints();
        if (points.empty()) return;

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl_cloud.reserve(points.size());
        for (const auto& p : points) {
            pcl_cloud.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
        }

        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(pcl_cloud, *msg);
        msg->header.stamp = now();
        msg->header.frame_id = map_frame_;
        map_pub_->publish(std::move(msg));
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<g1_msgs::msg::SlamStatus>::SharedPtr status_pub_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr map_timer_;

    // Core components
    std::unique_ptr<KissICPWrapper> kiss_icp_;
    std::unique_ptr<EKFFusion> ekf_;
    std::unique_ptr<LoopClosureDetector> loop_detector_;
    std::unique_ptr<PoseGraph> pose_graph_;

    // Config
    KissICPConfig icp_config_;
    LoopClosureDetector::Config lc_config_;
    std::string odom_frame_, base_frame_, map_frame_;
    double map_publish_rate_ = 1.0;
    bool enable_loop_closure_ = true;
    uint32_t loop_closure_count_ = 0;

    // Timestamp tracking for proper dt computation
    double last_imu_ts_ = 0.0;
    double last_leg_odom_ts_ = 0.0;
};

}  // namespace g1_slam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_slam::SlamNode>());
    rclcpp::shutdown();
    return 0;
}
