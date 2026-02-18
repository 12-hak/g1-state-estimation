#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "g1_msgs/srv/save_map.hpp"
#include "g1_msgs/srv/load_map.hpp"

#include <mutex>
#include <filesystem>
#include <fstream>

namespace g1_slam {

class MapManagerNode : public rclcpp::Node {
public:
    explicit MapManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("map_manager_node", options) {

        declare_parameter("map_directory", "/home/unitree/maps");
        declare_parameter("map_frame", "map");

        map_directory_ = get_parameter("map_directory").as_string();
        map_frame_ = get_parameter("map_frame").as_string();

        map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "slam/map", 1,
            std::bind(&MapManagerNode::mapCallback, this, std::placeholders::_1));

        grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).transient_local(),
            std::bind(&MapManagerNode::gridCallback, this, std::placeholders::_1));

        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("slam/map", rclcpp::QoS(1).transient_local());
        grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());

        save_srv_ = create_service<g1_msgs::srv::SaveMap>(
            "save_map",
            std::bind(&MapManagerNode::saveMapCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        load_srv_ = create_service<g1_msgs::srv::LoadMap>(
            "load_map",
            std::bind(&MapManagerNode::loadMapCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        std::filesystem::create_directories(map_directory_);
        RCLCPP_INFO(get_logger(), "Map manager ready (dir: %s)", map_directory_.c_str());
    }

private:
    void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        latest_map_ = *msg;
        has_map_ = true;
    }

    void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        latest_grid_ = *msg;
        has_grid_ = true;
    }

    void saveMapCallback(
        const std::shared_ptr<g1_msgs::srv::SaveMap::Request> request,
        std::shared_ptr<g1_msgs::srv::SaveMap::Response> response) {

        std::string name = request->filename;
        if (name.empty()) {
            auto t = std::chrono::system_clock::now();
            auto epoch = t.time_since_epoch();
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch).count();
            name = "map_" + std::to_string(seconds);
        }

        std::string pcd_path = map_directory_ + "/" + name + ".pcd";
        std::string grid_path = map_directory_ + "/" + name + "_grid.pgm";
        std::string yaml_path = map_directory_ + "/" + name + ".yaml";

        try {
            // Save point cloud map
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                if (!has_map_) {
                    response->success = false;
                    response->message = "No point cloud map available";
                    return;
                }
                pcl::PointCloud<pcl::PointXYZ> cloud;
                pcl::fromROSMsg(latest_map_, cloud);
                pcl::io::savePCDFileBinary(pcd_path, cloud);
            }

            // Save occupancy grid as PGM + YAML
            {
                std::lock_guard<std::mutex> lock(grid_mutex_);
                if (has_grid_) {
                    saveOccupancyGridPGM(latest_grid_, grid_path);
                    saveOccupancyGridYAML(latest_grid_, yaml_path, name + "_grid.pgm");
                }
            }

            response->success = true;
            response->message = "Map saved successfully";
            response->filepath = pcd_path;
            RCLCPP_INFO(get_logger(), "Map saved: %s", pcd_path.c_str());

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Save failed: ") + e.what();
            RCLCPP_ERROR(get_logger(), "Map save failed: %s", e.what());
        }
    }

    void loadMapCallback(
        const std::shared_ptr<g1_msgs::srv::LoadMap::Request> request,
        std::shared_ptr<g1_msgs::srv::LoadMap::Response> response) {

        try {
            std::string path = request->filepath;

            // Load PCD
            pcl::PointCloud<pcl::PointXYZ> cloud;
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud) == -1) {
                response->success = false;
                response->message = "Failed to load PCD file: " + path;
                return;
            }

            auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(cloud, *msg);
            msg->header.stamp = now();
            msg->header.frame_id = map_frame_;
            map_pub_->publish(std::move(msg));

            // Try loading corresponding grid
            std::string base = path.substr(0, path.find_last_of('.'));
            std::string yaml_path = base + ".yaml";
            if (std::filesystem::exists(yaml_path)) {
                loadOccupancyGridFromYAML(yaml_path);
            }

            response->success = true;
            response->message = "Map loaded: " + std::to_string(cloud.size()) + " points";
            RCLCPP_INFO(get_logger(), "Map loaded: %zu points from %s",
                        cloud.size(), path.c_str());

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Load failed: ") + e.what();
        }
    }

    void saveOccupancyGridPGM(const nav_msgs::msg::OccupancyGrid& grid,
                               const std::string& path) {
        std::ofstream pgm(path, std::ios::binary);
        pgm << "P5\n" << grid.info.width << " " << grid.info.height << "\n255\n";

        for (int y = static_cast<int>(grid.info.height) - 1; y >= 0; --y) {
            for (uint32_t x = 0; x < grid.info.width; ++x) {
                int8_t val = grid.data[y * grid.info.width + x];
                uint8_t pixel;
                if (val == -1) pixel = 205;       // Unknown = gray
                else if (val == 0) pixel = 254;    // Free = white
                else pixel = 0;                     // Occupied = black
                pgm.write(reinterpret_cast<char*>(&pixel), 1);
            }
        }
    }

    void saveOccupancyGridYAML(const nav_msgs::msg::OccupancyGrid& grid,
                                const std::string& yaml_path,
                                const std::string& image_file) {
        std::ofstream yaml(yaml_path);
        yaml << "image: " << image_file << "\n"
             << "resolution: " << grid.info.resolution << "\n"
             << "origin: [" << grid.info.origin.position.x << ", "
             << grid.info.origin.position.y << ", 0.0]\n"
             << "negate: 0\n"
             << "occupied_thresh: 0.65\n"
             << "free_thresh: 0.196\n";
    }

    void loadOccupancyGridFromYAML(const std::string&) {
        // Nav2 map_server handles YAML loading natively
        RCLCPP_INFO(get_logger(), "Grid YAML available for Nav2 map_server");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    rclcpp::Service<g1_msgs::srv::SaveMap>::SharedPtr save_srv_;
    rclcpp::Service<g1_msgs::srv::LoadMap>::SharedPtr load_srv_;

    sensor_msgs::msg::PointCloud2 latest_map_;
    nav_msgs::msg::OccupancyGrid latest_grid_;
    std::mutex map_mutex_, grid_mutex_;
    bool has_map_ = false, has_grid_ = false;

    std::string map_directory_;
    std::string map_frame_;
};

}  // namespace g1_slam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_slam::MapManagerNode>());
    rclcpp::shutdown();
    return 0;
}
