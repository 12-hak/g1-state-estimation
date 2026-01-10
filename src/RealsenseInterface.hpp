#pragma once

#include <librealsense2/rs.hpp> 
#include <iostream>
#include <mutex>
#include <vector>

class RealsenseInterface {
public:
    RealsenseInterface() {
        std::cout << "[RealsenseInterface] Initializing..." << std::endl;
        try {
            rs2::context ctx;
            auto devices = ctx.query_devices();
            if (devices.size() == 0) {
                std::cout << "[RealsenseInterface] Warning: No devices found." << std::endl;
                is_connected_ = false;
                return;
            }
            
            pipe_.start();
            is_connected_ = true;
            std::cout << "[RealsenseInterface] Pipeline started." << std::endl;
        } catch (const rs2::error & e) {
            std::cerr << "[RealsenseInterface] Error: " << e.what() << std::endl;
            is_connected_ = false;
        }
    }

    bool is_connected() const { return is_connected_; }

    void update() {
        if (!is_connected_) return;
        
        try {
            rs2::frameset frames = pipe_.wait_for_frames(100); // 100ms timeout
            // Process frames if needed
        } catch (...) {
            // Timeout or error
        }
    }

private:
    rs2::pipeline pipe_;
    bool is_connected_ = false;
};
