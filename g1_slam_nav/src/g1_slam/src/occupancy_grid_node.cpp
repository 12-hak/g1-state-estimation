#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

namespace g1_slam {

class OccupancyGridNode : public rclcpp::Node {
public:
    explicit OccupancyGridNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("occupancy_grid_node", options) {

        declare_parameter("resolution", 0.05);
        declare_parameter("width", 200.0);
        declare_parameter("height", 200.0);
        declare_parameter("min_obstacle_height", 0.15);
        declare_parameter("max_obstacle_height", 2.0);
        declare_parameter("map_frame", "map");
        declare_parameter("publish_rate", 1.0);
        declare_parameter("free_threshold", 0.3);
        declare_parameter("occupied_threshold", 0.50);

        resolution_ = get_parameter("resolution").as_double();
        width_ = get_parameter("width").as_double();
        height_ = get_parameter("height").as_double();
        min_obstacle_height_ = get_parameter("min_obstacle_height").as_double();
        max_obstacle_height_ = get_parameter("max_obstacle_height").as_double();
        map_frame_ = get_parameter("map_frame").as_string();
        free_threshold_ = get_parameter("free_threshold").as_double();
        occupied_threshold_ = get_parameter("occupied_threshold").as_double();

        grid_width_ = static_cast<int>(width_ / resolution_);
        grid_height_ = static_cast<int>(height_ / resolution_);
        origin_x_ = -width_ / 2.0;
        origin_y_ = -height_ / 2.0;

        // Log-odds grid for Bayesian updates
        log_odds_.resize(grid_width_ * grid_height_, 0.0);

        map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "slam/map", 1,
            std::bind(&OccupancyGridNode::mapCallback, this, std::placeholders::_1));

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "slam/pose", 10,
            std::bind(&OccupancyGridNode::poseCallback, this, std::placeholders::_1));

        grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());

        double rate = get_parameter("publish_rate").as_double();
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&OccupancyGridNode::publishGrid, this));

        RCLCPP_INFO(get_logger(), "Occupancy grid: %dx%d @ %.2fm (obstacle height: %.2f-%.2fm)",
                    grid_width_, grid_height_, resolution_,
                    min_obstacle_height_, max_obstacle_height_);
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        robot_x_ = msg->pose.position.x;
        robot_y_ = msg->pose.position.y;
    }

    void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        std::lock_guard<std::mutex> lock(grid_mutex_);

        for (const auto& p : cloud) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

            // Height filtering for obstacle detection
            if (p.z < min_obstacle_height_ || p.z > max_obstacle_height_) continue;

            int gx = static_cast<int>((p.x - origin_x_) / resolution_);
            int gy = static_cast<int>((p.y - origin_y_) / resolution_);

            if (gx < 0 || gx >= grid_width_ || gy < 0 || gy >= grid_height_) continue;

            int idx = gy * grid_width_ + gx;

            // Bayesian update (log-odds): increase probability of occupied
            log_odds_[idx] = std::min(log_odds_[idx] + 1.5, 10.0);
        }

        // Ray-casting for free space (from robot position to each point)
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
            if (pt_idx % 4 != 0) continue; // Downsample ray-casting to save CPU
            
            if (p.z < min_obstacle_height_ || p.z > max_obstacle_height_) continue;

            int end_gx = static_cast<int>((p.x - origin_x_) / resolution_);
            int end_gy = static_cast<int>((p.y - origin_y_) / resolution_);

            // Bresenham ray from robot to point (mark cells as free)
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

        map_updated_ = true;
    }

    void publishGrid() {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        if (!map_updated_) return;

        auto msg = nav_msgs::msg::OccupancyGrid();
        msg.header.stamp = now();
        msg.header.frame_id = map_frame_;

        msg.info.resolution = resolution_;
        msg.info.width = grid_width_;
        msg.info.height = grid_height_;
        msg.info.origin.position.x = origin_x_;
        msg.info.origin.position.y = origin_y_;
        msg.info.origin.orientation.w = 1.0;

        msg.data.resize(grid_width_ * grid_height_);
        for (size_t i = 0; i < log_odds_.size(); ++i) {
            double prob = 1.0 / (1.0 + std::exp(-log_odds_[i]));
            if (prob > occupied_threshold_) {
                msg.data[i] = 100;
            } else if (prob < free_threshold_) {
                msg.data[i] = 0;
            } else {
                msg.data[i] = -1;  // Unknown
            }
        }

        grid_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> log_odds_;
    std::mutex grid_mutex_;
    std::mutex pose_mutex_;

    double robot_x_ = 0.0, robot_y_ = 0.0;
    bool map_updated_ = false;

    double resolution_;
    double width_, height_;
    uint32_t grid_width_, grid_height_;
    double origin_x_, origin_y_;
    double min_obstacle_height_, max_obstacle_height_;
    double free_threshold_, occupied_threshold_;
    std::string map_frame_;
};

}  // namespace g1_slam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_slam::OccupancyGridNode>());
    rclcpp::shutdown();
    return 0;
}
