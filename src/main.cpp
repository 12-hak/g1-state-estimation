#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>
#include <csignal>

#include "UnitreeInterface.hpp"
#include "StateEstimator.hpp"
#include "RealsenseInterface.hpp"
#include "LivoxInterface.hpp"
#include "StatePublisher.hpp" // New

std::atomic<bool> running(true);

void signal_handler(int signum) {
    running = false;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    std::cout << "[G1 State Estimator] Starting..." << std::endl;
    
    std::string network_interface = "eth0";
    if (argc > 1) {
        network_interface = argv[1];
    }

    // Initialize Sensors
    UnitreeInterface robot(network_interface);
    RealsenseInterface camera;
    LivoxInterface lidar;

    // Initialize Estimator & Publisher
    StateEstimator estimator;
    StatePublisher publisher("g1_state_shm");

    // Main Loop
    while (running) {
        auto start = std::chrono::high_resolution_clock::now();

        // 1. Get Sensor Data
        G1SensorData robot_data = robot.get_latest_data();
        
        // 2. Update Estimator
        estimator.update_imu(robot_data.quaternion, robot_data.gyroscope, robot_data.accelerometer);
        estimator.update_joints(robot_data.q, robot_data.dq, robot_data.tau);
        
        // Stub: Update from other sensors
        // camera.update();
        // lidar.update();

        // 3. Publish State
        RobotState state = estimator.get_state();
        publisher.publish(state);
        
        // Debug Print (Every 100 ticks)
        static int tick = 0;
        if (tick++ % 100 == 0) {
            std::cout << "Pos: " << state.position.transpose() << " | Vel: " << state.velocity.transpose() << std::endl;
        }

        // Rate Limiting (e.g. 500Hz)
        std::this_thread::sleep_until(start + std::chrono::microseconds(2000));
    }

    std::cout << "[G1 State Estimator] Shutting down." << std::endl;
    return 0;
}
