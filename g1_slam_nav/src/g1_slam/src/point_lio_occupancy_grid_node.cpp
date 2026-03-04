/**
 * Occupancy grid from Point-LIO: subscribes to Laser_map (PointCloud2) and
 * aft_mapped_to_init (Odometry), publishes /map (OccupancyGrid) for Nav2.
 * Same logic as occupancy_grid_node but with Point-LIO topic names and Odometry pose.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

namespace g1_slam {

class PointLioOccupancyGridNode : public rclcpp::Node {
public:
    explicit PointLioOccupancyGridNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("point_lio_occupancy_grid_node", options) {

        declare_parameter("map_topic", std::string("Laser_map"));
        declare_parameter("odom_topic", std::string("aft_mapped_to_init"));
        declare_parameter("resolution", 0.05);
        declare_parameter("width", 200.0);
        declare_parameter("height", 200.0);
        declare_parameter("min_obstacle_height", 0.15);
        declare_parameter("max_obstacle_height", 2.0);
        declare_parameter("map_frame", std::string("camera_init"));
        declare_parameter("publish_rate", 1.0);
        declare_parameter("free_threshold", 0.3);
        declare_parameter("occupied_threshold", 0.50);
        declare_parameter("rolling_window", true);

        std::string map_topic = get_parameter("map_topic").as_string();
        std::string odom_topic = get_parameter("odom_topic").as_string();
        resolution_ = get_parameter("resolution").as_double();
        width_ = get_parameter("width").as_double();
        height_ = get_parameter("height").as_double();
        min_obstacle_height_ = get_parameter("min_obstacle_height").as_double();
        max_obstacle_height_ = get_parameter("max_obstacle_height").as_double();
        map_frame_ = get_parameter("map_frame").as_string();
        free_threshold_ = get_parameter("free_threshold").as_double();
        occupied_threshold_ = get_parameter("occupied_threshold").as_double();
        rolling_window_ = get_parameter("rolling_window").as_bool();

        grid_width_ = static_cast<int>(width_ / resolution_);
        grid_height_ = static_cast<int>(height_ / resolution_);
        origin_x_ = -width_ / 2.0;
        origin_y_ = -height_ / 2.0;

        log_odds_.resize(grid_width_ * grid_height_, 0.0);

        map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            map_topic, 1,
            std::bind(&PointLioOccupancyGridNode::mapCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&PointLioOccupancyGridNode::odomCallback, this, std::placeholders::_1));

        grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());

        double rate = get_parameter("publish_rate").as_double();
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&PointLioOccupancyGridNode::publishGrid, this));

        RCLCPP_INFO(get_logger(), "Point-LIO occupancy grid: %s + %s -> /map (%dx%d @ %.2fm)",
                    map_topic.c_str(), odom_topic.c_str(),
                    grid_width_, grid_height_, resolution_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
    }

    void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        if (rolling_window_) {
            std::lock_guard<std::mutex> lock(grid_mutex_);
            last_cloud_ = cloud;
            map_updated_ = true;
            return;
        }

        std::lock_guard<std::mutex> lock(grid_mutex_);

        for (const auto& p : cloud) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            if (p.z < min_obstacle_height_ || p.z > max_obstacle_height_) continue;

            int gx = static_cast<int>((p.x - origin_x_) / resolution_);
            int gy = static_cast<int>((p.y - origin_y_) / resolution_);
            if (gx < 0 || gx >= grid_width_ || gy < 0 || gy >= grid_height_) continue;

            int idx = gy * grid_width_ + gx;
            log_odds_[idx] = std::min(log_odds_[idx] + 1.5, 10.0);
        }

        if (!rolling_window_) {
            double rx, ry;
            {
                std::lock_guard<std::mutex> lock2(pose_mutex_);
                rx = robot_x_;
                ry = robot_y_;
            }

            int robot_gx = static_cast<int>((rx - origin_x_) / resolution_);
            int robot_gy = static_cast<int>((ry - origin_y_) / resolution_);

            int pt_idx = 0;
            for (const auto& p : cloud) {
                pt_idx++;
                if (pt_idx % 4 != 0) continue;
                if (p.z < min_obstacle_height_ || p.z > max_obstacle_height_) continue;

                int end_gx = static_cast<int>((p.x - origin_x_) / resolution_);
                int end_gy = static_cast<int>((p.y - origin_y_) / resolution_);

                int dx = std::abs(end_gx - robot_gx);
                int dy = std::abs(end_gy - robot_gy);
                int sx = (robot_gx < end_gx) ? 1 : -1;
                int sy = (robot_gy < end_gy) ? 1 : -1;
                int err = dx - dy;
                int cx = robot_gx, cy = robot_gy;
                int max_steps = dx + dy;
                for (int step = 0; step < max_steps - 1; ++step) {
                    if (cx >= 0 && cx < grid_width_ && cy >= 0 && cy < grid_height_) {
                        int idx = cy * grid_width_ + cx;
                        log_odds_[idx] = std::max(log_odds_[idx] - 0.3, -3.0);
                    }
                    int e2 = 2 * err;
                    if (e2 > -dy) { err -= dy; cx += sx; }
                    if (e2 < dx)  { err += dx; cy += sy; }
                }
            }
        }

        map_updated_ = true;
    }

    void buildGridFromCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, double ox, double oy) {
        std::fill(log_odds_.begin(), log_odds_.end(), 0.0);
        for (const auto& p : cloud) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            if (p.z < min_obstacle_height_ || p.z > max_obstacle_height_) continue;
            int gx = static_cast<int>((p.x - ox) / resolution_);
            int gy = static_cast<int>((p.y - oy) / resolution_);
            if (gx < 0 || gx >= static_cast<int>(grid_width_) || gy < 0 || gy >= static_cast<int>(grid_height_)) continue;
            int idx = gy * grid_width_ + gx;
            log_odds_[idx] = std::min(log_odds_[idx] + 1.5, 10.0);
        }
        double rx, ry;
        {
            std::lock_guard<std::mutex> lock2(pose_mutex_);
            rx = robot_x_;
            ry = robot_y_;
        }
        int robot_gx = static_cast<int>((rx - ox) / resolution_);
        int robot_gy = static_cast<int>((ry - oy) / resolution_);
        int pt_idx = 0;
        for (const auto& p : cloud) {
            pt_idx++;
            if (pt_idx % 4 != 0) continue;
            if (p.z < min_obstacle_height_ || p.z > max_obstacle_height_) continue;
            int end_gx = static_cast<int>((p.x - ox) / resolution_);
            int end_gy = static_cast<int>((p.y - oy) / resolution_);
            int dx = std::abs(end_gx - robot_gx);
            int dy = std::abs(end_gy - robot_gy);
            int sx = (robot_gx < end_gx) ? 1 : -1;
            int sy = (robot_gy < end_gy) ? 1 : -1;
            int err = dx - dy;
            int cx = robot_gx, cy = robot_gy;
            int max_steps = dx + dy;
            for (int step = 0; step < max_steps - 1; ++step) {
                if (cx >= 0 && cx < static_cast<int>(grid_width_) && cy >= 0 && cy < static_cast<int>(grid_height_)) {
                    int idx = cy * grid_width_ + cx;
                    log_odds_[idx] = std::max(log_odds_[idx] - 0.3, -3.0);
                }
                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; cx += sx; }
                if (e2 < dx)  { err += dx; cy += sy; }
            }
        }
    }

    void publishGrid() {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        if (!map_updated_) return;
        if (rolling_window_ && last_cloud_.empty()) return;

        double pub_origin_x = origin_x_, pub_origin_y = origin_y_;
        if (rolling_window_) {
            double rx, ry;
            {
                std::lock_guard<std::mutex> lock2(pose_mutex_);
                rx = robot_x_;
                ry = robot_y_;
            }
            pub_origin_x = rx - width_ / 2.0;
            pub_origin_y = ry - height_ / 2.0;
            buildGridFromCloud(last_cloud_, pub_origin_x, pub_origin_y);
        }

        auto msg = nav_msgs::msg::OccupancyGrid();
        msg.header.stamp = now();
        msg.header.frame_id = map_frame_;
        msg.info.resolution = resolution_;
        msg.info.width = grid_width_;
        msg.info.height = grid_height_;
        msg.info.origin.position.x = pub_origin_x;
        msg.info.origin.position.y = pub_origin_y;
        msg.info.origin.orientation.w = 1.0;
        msg.data.resize(grid_width_ * grid_height_);
        for (size_t i = 0; i < log_odds_.size(); ++i) {
            double prob = 1.0 / (1.0 + std::exp(-log_odds_[i]));
            if (prob > occupied_threshold_)
                msg.data[i] = 100;
            else if (prob < free_threshold_)
                msg.data[i] = 0;
            else
                msg.data[i] = -1;
        }
        grid_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> log_odds_;
    std::mutex grid_mutex_, pose_mutex_;
    double robot_x_ = 0.0, robot_y_ = 0.0;
    bool map_updated_ = false;
    bool rolling_window_ = true;
    pcl::PointCloud<pcl::PointXYZ> last_cloud_;
    double resolution_, width_, height_;
    uint32_t grid_width_, grid_height_;
    double origin_x_, origin_y_;
    double min_obstacle_height_, max_obstacle_height_;
    double free_threshold_, occupied_threshold_;
    std::string map_frame_;
};

}  // namespace g1_slam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_slam::PointLioOccupancyGridNode>());
    rclcpp::shutdown();
    return 0;
}
