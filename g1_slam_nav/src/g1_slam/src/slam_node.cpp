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
        buildLidarToBase();

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
        registered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "slam/registered_points", 5);
        status_pub_ = create_publisher<g1_msgs::msg::SlamStatus>("slam/status", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        latest_lin_vel_.setZero();
        latest_ang_vel_.setZero();

        auto map_period = std::chrono::duration<double>(1.0 / map_publish_rate_);
        map_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(map_period),
            std::bind(&SlamNode::publishMap, this));

        RCLCPP_INFO(get_logger(),
            "SLAM node initialized (voxel=%.2f, range=%.1f-%.1f, lidar_offset=[%.2f,%.2f,%.2f] flip=%s)",
            icp_config_.voxel_size, icp_config_.min_range, icp_config_.max_range,
            lidar_offset_x_, lidar_offset_y_, lidar_offset_z_,
            flip_lidar_ ? "true" : "false");
    }

private:
    void declareParameters() {
        declare_parameter("voxel_size", 0.4);
        declare_parameter("max_range", 15.0);
        declare_parameter("min_range", 0.3);
        declare_parameter("max_icp_iterations", 30);
        declare_parameter("odom_frame", "odom");
        declare_parameter("base_frame", "base_link");
        declare_parameter("map_frame", "map");
        declare_parameter("map_publish_rate", 1.0);
        declare_parameter("keyframe_distance", 1.0);
        declare_parameter("keyframe_angle", 0.5);
        declare_parameter("loop_search_radius", 10.0);
        declare_parameter("enable_loop_closure", true);
        declare_parameter("lidar_offset_x", 0.10);
        declare_parameter("lidar_offset_y", 0.0);
        declare_parameter("lidar_offset_z", 0.60);
        declare_parameter("flip_lidar", true);

        icp_config_.voxel_size = get_parameter("voxel_size").as_double();
        icp_config_.max_range = get_parameter("max_range").as_double();
        icp_config_.min_range = get_parameter("min_range").as_double();
        icp_config_.max_iterations = get_parameter("max_icp_iterations").as_int();
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        map_frame_ = get_parameter("map_frame").as_string();
        map_publish_rate_ = get_parameter("map_publish_rate").as_double();
        enable_loop_closure_ = get_parameter("enable_loop_closure").as_bool();
        lidar_offset_x_ = get_parameter("lidar_offset_x").as_double();
        lidar_offset_y_ = get_parameter("lidar_offset_y").as_double();
        lidar_offset_z_ = get_parameter("lidar_offset_z").as_double();
        flip_lidar_ = get_parameter("flip_lidar").as_bool();

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

    void buildLidarToBase() {
        // The livox_bridge already flips Y/Z in point data when flip_lidar=true,
        // so incoming points are orientation-correct but still centered at the
        // lidar origin. We only need to apply the translation offset.
        lidar_to_base_ = Eigen::Matrix4d::Identity();
        lidar_to_base_(0,3) = lidar_offset_x_;
        lidar_to_base_(1,3) = lidar_offset_y_;
        lidar_to_base_(2,3) = lidar_offset_z_;

        lidar_R_ = lidar_to_base_.block<3,3>(0,0);  // Identity
        lidar_t_ = lidar_to_base_.block<3,1>(0,3);   // [0.10, 0, 0.60]
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        const auto wall_now = std::chrono::steady_clock::now();
        if (has_last_slam_wall_tp_) {
            double wall_dt = std::chrono::duration<double>(
                wall_now - last_slam_wall_tp_).count();
            if (wall_dt < 0.06) return;
        }
        last_slam_wall_tp_ = wall_now;
        has_last_slam_wall_tp_ = true;

        // Extract points and transform lidar_link → base_link
        std::vector<Eigen::Vector3d> points;
        const int step = msg->point_step;
        const int count = msg->width * msg->height;
        points.reserve(count / 2);

        const uint8_t* ptr = msg->data.data();
        for (int i = 0; i < count; i += 2) {
            float x, y, z;
            memcpy(&x, ptr + i * step + 0, 4);
            memcpy(&y, ptr + i * step + 4, 4);
            memcpy(&z, ptr + i * step + 8, 4);

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            // Transform from lidar frame to base_link frame
            Eigen::Vector3d p_lidar(x, y, z);
            Eigen::Vector3d p_base = lidar_R_ * p_lidar + lidar_t_;
            points.push_back(p_base);
        }

        if (points.empty()) return;

        // Build the ICP guess. Rotation comes entirely from odometry (IMU-fused),
        // translation starts from the last ICP position + odom delta.
        // ICP then refines ONLY the translation against the map.
        Eigen::Matrix4d guess = Eigen::Matrix4d::Identity();
        double odom_yaw = std::atan2(
            latest_odom_pose_(1,0), latest_odom_pose_(0,0));

        if (has_guess_seed_) {
            Eigen::Matrix4d odom_delta =
                prev_odom_for_guess_.inverse() * latest_odom_pose_;
            Eigen::Vector3d dt = odom_delta.block<3,1>(0,3);

            // Translation: last ICP position + odom displacement
            Eigen::Vector3d new_t = prev_icp_for_guess_.block<3,1>(0,3) + 
                prev_icp_for_guess_.block<3,3>(0,0) * dt;
            guess.block<3,1>(0,3) = new_t;
        } else {
            guess.block<3,1>(0,3) = kiss_icp_->currentPose().block<3,1>(0,3);
        }

        // Rotation: always from odometry (reliable from IMU)
        guess.block<3,3>(0,0) = latest_odom_pose_.block<3,3>(0,0);

        auto result = kiss_icp_->registerFrame(points, guess);

        double result_yaw = std::atan2(result.pose(1,0), result.pose(0,0));

        if (!result.converged) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "ICP FAIL: fit=%.4f corr=%d iter=%d odom_yaw=%.1f",
                result.fitness_score, result.num_correspondences,
                result.iterations, odom_yaw * 180.0 / M_PI);
            return;
        }

        // Reject implausible translation jumps
        if (has_prev_icp_pose_) {
            double dtrans = (result.pose.block<3,1>(0,3) - 
                prev_icp_pose_.block<3,1>(0,3)).norm();
            if (dtrans > 0.5) {
                RCLCPP_WARN(get_logger(),
                    "ICP REJECT: dt=%.3f fit=%.4f corr=%d",
                    dtrans, result.fitness_score,
                    result.num_correspondences);
                return;
            }
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "SLAM: pos=(%.2f,%.2f) yaw=%.1f fit=%.4f corr=%d map=%zu",
            result.pose(0,3), result.pose(1,3),
            result_yaw * 180.0 / M_PI,
            result.fitness_score, result.num_correspondences,
            kiss_icp_->localMap().size());

        prev_icp_pose_ = result.pose;
        has_prev_icp_pose_ = true;
        prev_odom_for_guess_ = latest_odom_pose_;
        prev_icp_for_guess_ = result.pose;
        has_guess_seed_ = true;

        // Feed ICP result to EKF
        Eigen::Matrix<double, 6, 6> lidar_cov =
            Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
        lidar_cov(2,2) = 0.05;
        lidar_cov(5,5) = 0.001;
        ekf_->updateLidar(result.pose, lidar_cov);

        Eigen::Matrix4d fused = ekf_->getPose();
        auto stamp = msg->header.stamp;

        publishPose(result.pose, stamp);
        publishOdometry(result.pose, stamp);
        publishTF(result.pose, latest_odom_pose_, stamp);
        publishRegisteredPoints(points, result.pose, stamp);

        // Loop closure
        if (enable_loop_closure_) {
            double ts = stamp.sec + stamp.nanosec * 1e-9;
            if (loop_detector_->addKeyframe(fused, points, ts)) {
                int kf_id = loop_detector_->numKeyframes() - 1;
                pose_graph_->addNode(kf_id, fused);

                if (kf_id > 0) {
                    Eigen::Matrix4d prev = pose_graph_->getNodePose(kf_id - 1);
                    Eigen::Matrix4d rel = prev.inverse() * fused;
                    Eigen::Matrix<double, 6, 6> odom_info =
                        Eigen::Matrix<double, 6, 6>::Identity() * 100.0;
                    pose_graph_->addEdge(kf_id - 1, kf_id, rel, odom_info);
                }

                auto loop = loop_detector_->detectLoop();
                if (loop.has_value()) {
                    RCLCPP_INFO(get_logger(),
                        "Loop closure: %d->%d (score=%.4f)",
                        loop->from_id, loop->to_id, loop->score);
                    pose_graph_->addEdge(loop->from_id, loop->to_id,
                        loop->relative_pose, loop->information);

                    if (pose_graph_->optimize(10)) {
                        Eigen::Matrix4d corrected =
                            pose_graph_->getNodePose(kf_id);
                        ekf_->setPose(corrected);
                        kiss_icp_->setPose(corrected);
                        prev_icp_for_guess_ = corrected;
                        prev_odom_for_guess_ = latest_odom_pose_;
                        has_guess_seed_ = true;
                        loop_closure_count_++;
                    }
                }
            }
        }

        auto status = g1_msgs::msg::SlamStatus();
        status.header.stamp = stamp;
        status.mapping_active = true;
        status.localization_valid = result.converged;
        status.map_point_count =
            static_cast<uint32_t>(kiss_icp_->localMap().size());
        status.loop_closure_count = loop_closure_count_;
        status.odometry_quality = 1.0 - std::min(result.fitness_score, 1.0);
        status.map_resolution = icp_config_.voxel_size;
        status_pub_->publish(status);
    }

    void predict(double current_ts) {
        if (last_predict_ts_ <= 0) {
            last_predict_ts_ = current_ts;
            return;
        }
        double dt = current_ts - last_predict_ts_;
        if (dt <= 0) return;
        if (dt > 0.1) dt = 0.02;
        ekf_->predict(latest_lin_vel_, latest_ang_vel_, dt);
        last_predict_ts_ = current_ts;
    }

    void legOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double current_ts =
            msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        latest_lin_vel_ = Eigen::Vector3d(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z);
        if (current_ts - last_imu_ts_ > 0.1) {
            latest_ang_vel_ = Eigen::Vector3d(
                msg->twist.twist.angular.x,
                msg->twist.twist.angular.y,
                msg->twist.twist.angular.z);
        }

        predict(current_ts);

        Eigen::Vector3d pos(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);

        latest_odom_pose_ = Eigen::Matrix4d::Identity();
        latest_odom_pose_.block<3,3>(0,0) = q.toRotationMatrix();
        latest_odom_pose_.block<3,1>(0,3) = pos;

        Eigen::Matrix<double, 6, 6> cov =
            Eigen::Matrix<double, 6, 6>::Identity();
        for (int i = 0; i < 6; ++i)
            cov(i,i) = msg->pose.covariance[i * 7];
        ekf_->updateLegOdom(pos, q, cov);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double current_ts =
            msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        last_imu_ts_ = current_ts;
        latest_ang_vel_ = Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
        predict(current_ts);
    }

    void publishPose(const Eigen::Matrix4d& pose,
                     const rclcpp::Time& stamp) {
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

    void publishOdometry(const Eigen::Matrix4d& pose,
                         const rclcpp::Time& stamp) {
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
        auto cov = ekf_->getCovariance();
        for (int i = 0; i < 6; ++i)
            msg.pose.covariance[i * 7] = cov(i, i);
        odom_pub_->publish(msg);
    }

    void publishTF(const Eigen::Matrix4d& world_pose,
                   const Eigen::Matrix4d& odom_pose,
                   const rclcpp::Time& stamp) {
        Eigen::Matrix4d map_to_odom = world_pose * odom_pose.inverse();
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = map_frame_;
        tf.child_frame_id = odom_frame_;
        tf.transform.translation.x = map_to_odom(0,3);
        tf.transform.translation.y = map_to_odom(1,3);
        tf.transform.translation.z = map_to_odom(2,3);
        Eigen::Quaterniond q(map_to_odom.block<3,3>(0,0));
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
        for (const auto& p : points)
            pcl_cloud.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(pcl_cloud, *msg);
        msg->header.stamp = now();
        msg->header.frame_id = map_frame_;
        map_pub_->publish(std::move(msg));
    }

    void publishRegisteredPoints(
            const std::vector<Eigen::Vector3d>& base_points,
            const Eigen::Matrix4d& pose,
            const rclcpp::Time& stamp) {
        if (base_points.empty()) return;
        Eigen::Matrix3d R = pose.block<3,3>(0,0);
        Eigen::Vector3d t = pose.block<3,1>(0,3);
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        size_t stride = std::max(
            static_cast<size_t>(1), base_points.size() / 800);
        pcl_cloud.reserve(base_points.size() / stride);
        for (size_t i = 0; i < base_points.size(); i += stride) {
            Eigen::Vector3d pw = R * base_points[i] + t;
            pcl_cloud.push_back(pcl::PointXYZ(pw.x(), pw.y(), pw.z()));
        }
        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(pcl_cloud, *msg);
        msg->header.stamp = stamp;
        msg->header.frame_id = map_frame_;
        registered_pub_->publish(std::move(msg));
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_pub_;
    rclcpp::Publisher<g1_msgs::msg::SlamStatus>::SharedPtr status_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr map_timer_;

    // Core SLAM components
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

    // LiDAR extrinsics (lidar_link → base_link)
    double lidar_offset_x_ = 0.10;
    double lidar_offset_y_ = 0.0;
    double lidar_offset_z_ = 0.60;
    bool flip_lidar_ = true;
    Eigen::Matrix4d lidar_to_base_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d lidar_R_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d lidar_t_ = Eigen::Vector3d::Zero();

    // Timing
    double last_imu_ts_ = 0.0;
    double last_predict_ts_ = 0.0;
    std::chrono::steady_clock::time_point last_slam_wall_tp_;
    bool has_last_slam_wall_tp_ = false;

    // ICP state
    Eigen::Matrix4d prev_icp_pose_ = Eigen::Matrix4d::Identity();
    bool has_prev_icp_pose_ = false;
    Eigen::Matrix4d prev_odom_for_guess_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d prev_icp_for_guess_ = Eigen::Matrix4d::Identity();
    bool has_guess_seed_ = false;

    Eigen::Matrix4d latest_odom_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d latest_lin_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_ang_vel_ = Eigen::Vector3d::Zero();
};

}  // namespace g1_slam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_slam::SlamNode>());
    rclcpp::shutdown();
    return 0;
}
