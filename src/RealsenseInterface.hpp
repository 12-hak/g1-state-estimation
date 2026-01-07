#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <mutex>
#include <array>

class RealsenseInterface {
public:
    RealsenseInterface() {
        std::cout << "[RealsenseInterface] Initializing stub..." << std::endl;
        // context_ = std::make_unique<rs2::context>();
    }

    // Stub for getting Visual Odometry pose or depth
    // void update() {}

private:
    // std::unique_ptr<rs2::context> context_;
};
