#include "RealsenseInterface.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::cout << "[Test Realsense] Starting..." << std::endl;
    
    RealsenseInterface rs;
    
    if (!rs.is_connected()) {
        std::cout << "[Test Realsense] No device found or could not start pipeline." << std::endl;
        return -1;
    }

    std::cout << "[Test Realsense] Collecting frames for 2 seconds..." << std::endl;
    for (int i = 0; i < 20; ++i) {
        rs.update();
        std::cout << "." << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\n[Test Realsense] Success." << std::endl;
    return 0;
}
