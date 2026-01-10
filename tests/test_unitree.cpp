#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include "UnitreeInterface.hpp"

int main(int argc, char** argv) {
    std::string interface = "eth0";
    if (argc > 1) interface = argv[1];

    std::cout << "[Test Unitree] Initializing on " << interface << "..." << std::endl;
    UnitreeInterface robot(interface);

    std::cout << "Waiting for data from G1 (rt/lowstate)..." << std::endl;

    for (int i = 0; i < 50; ++i) {
        G1SensorData data = robot.get_latest_data();
        
        // Print IMU Data
        std::cout << "\r[IMU] Quat: [" 
                  << std::fixed << std::setprecision(2)
                  << data.quaternion[0] << ", " 
                  << data.quaternion[1] << ", "
                  << data.quaternion[2] << ", "
                  << data.quaternion[3] << "] "
                  << " | Joint[0] q: " << data.q[0] << "    " << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\n[Test Unitree] Finished." << std::endl;
    return 0;
}
