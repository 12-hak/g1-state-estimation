#include "G1Localizer.hpp"
#include "SimpleTCPServer.hpp"
#include <iostream>
#include <csignal>
#include <atomic>
#include <sstream>
#include <iomanip>

std::atomic<bool> g_running(true);

void signalHandler(int signal) {
    std::cout << "\nShutting down..." << std::endl;
    g_running = false;
}

// Helper to serialize vector of points
std::string vec2_to_json(const std::vector<Eigen::Vector2f>& points) {
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < points.size(); ++i) {
        ss << "[" << std::fixed << std::setprecision(3) << points[i].x() << "," << points[i].y() << "]";
        if (i < points.size() - 1) ss << ",";
    }
    ss << "]";
    return ss.str();
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::string network_interface = "eth0";
    // We don't care about the UDP receiver IP for the web viz, 
    // but we pass it anyway to satisfy G1Localizer constructor.
    std::string receiver_ip = "127.0.0.1"; 
    
    if (argc >= 2) network_interface = argv[1];

    std::cout << "G1 Web Localization Node" << std::endl;
    std::cout << "Starting Localizer..." << std::endl;
    
    try {
        g1_localization::G1Localizer localizer(network_interface, receiver_ip);
        localizer.start();
        
        g1_localization::SimpleTCPServer web_server(5000);
        web_server.start();
        
        std::cout << "Web Streaming Server running on port 5000" << std::endl;
        
        int frame_count = 0;
        
        while (g_running) {
            auto start = std::chrono::steady_clock::now();
            
            // Get Data
            auto state = localizer.getState();
            auto scan = localizer.getLatestScan();
            
            std::stringstream json;
            json << "{";
            
            // Pose
            json << "\"pos\":[" 
                 << state.position.x() << "," 
                 << state.position.y() << "," 
                 << state.position.z() << "],";
                 
            json << "\"yaw\":" << std::atan2(2.0f*(state.orientation.w()*state.orientation.z() + state.orientation.x()*state.orientation.y()), 
                                              1.0f - 2.0f*(state.orientation.y()*state.orientation.y() + state.orientation.z()*state.orientation.z())) << ",";

            // Correction Data (For debugging "wandering")
            json << "\"raw_pos\":[" 
                 << state.position_raw.x() << "," 
                 << state.position_raw.y() << "],";
            
            json << "\"icp_valid\":" << (state.icp_valid ? "true" : "false") << ",";
            json << "\"icp_error\":" << state.icp_error << ",";

            // Scan
            json << "\"scan\":" << vec2_to_json(scan);
            
            if (frame_count % 20 == 0) {
                auto map = localizer.getGlobalMap();
                json << ",\"map\":" << vec2_to_json(map);
            }
            
            json << "}";
            
            web_server.broadcast(json.str());
            
            frame_count++;
            
            // 20Hz Loop
            std::this_thread::sleep_until(start + std::chrono::milliseconds(50));
        }
        
        web_server.stop();
        localizer.stop();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
