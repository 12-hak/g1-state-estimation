#include "G1Localizer.hpp"
#include <iostream>
#include <csignal>
#include <atomic>

std::atomic<bool> g_running(true);

void signalHandler(int signal) {
    std::cout << "\nShutting down..." << std::endl;
    g_running = false;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::string network_interface = "eth0";
    std::string receiver_ip = "192.168.123.100";  // Default PC IP
    
    if (argc >= 2) {
        network_interface = argv[1];
    }
    if (argc >= 3) {
        receiver_ip = argv[2];
    }
    
    std::cout << "==================================" << std::endl;
    std::cout << "G1 Localization Node (DDS)" << std::endl;
    std::cout << "==================================" << std::endl;
    std::cout << "Network Interface: " << network_interface << std::endl;
    std::cout << "Receiver IP: " << receiver_ip << ":9870" << std::endl;
    std::cout << std::endl;
    
    try {
        g1_localization::G1Localizer localizer(network_interface, receiver_ip);
        localizer.start();
        
        std::cout << "Localization running. Press Ctrl+C to stop." << std::endl;
        std::cout << "Sending state to visualizer at " << receiver_ip << ":9870" << std::endl;
        
        // Main loop - print state periodically
        auto last_print = std::chrono::steady_clock::now();
        
        while (g_running) {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_print);
            
            if (duration.count() >= 1) {
                auto state = localizer.getState();
                
                /*
                std::cout << "[State] "
                          << "Pos: (" << state.position.x() << ", " << state.position.y() << ", " << state.position.z() << ") "
                          << "Vel: (" << state.velocity.x() << ", " << state.velocity.y() << ") "
                          << "ICP: " << (state.icp_valid ? "OK" : "FAIL") << " "
                          << "Err: " << state.icp_error
                          << std::endl;
                */
                
                last_print = now;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        localizer.stop();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "Shutdown complete." << std::endl;
    return 0;
}
