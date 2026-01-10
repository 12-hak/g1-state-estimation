#include "LivoxInterface.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::cout << "[Test Livox] Starting..." << std::endl;
    
    LivoxInterface lidar;
    
    if (!lidar.is_initialized()) {
        std::cout << "[Test Livox] Failed to initialize Livox SDK." << std::endl;
        return -1;
    }

    std::cout << "[Test Livox] Listening for 3 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "[Test Livox] Success." << std::endl;
    return 0;
}
