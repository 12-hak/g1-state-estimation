#include "LiDARRecorder.hpp"
#include <iostream>
#include <csignal>

using namespace g1_localization;

std::unique_ptr<LiDARRecorder> recorder;

void signalHandler(int signum) {
    std::cout << "\n[Main] Interrupt signal received. Stopping..." << std::endl;
    if (recorder) recorder->stop();
    exit(0);
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::string output_dir = argc > 1 ? argv[1] : "./recordings";
    std::string localizer_ip = argc > 2 ? argv[2] : "127.0.0.1";
    uint16_t localizer_port = argc > 3 ? std::stoi(argv[3]) : 9871;
    
    std::cout << "=== G1 LiDAR Recorder ===" << std::endl;
    std::cout << "Output directory: " << output_dir << std::endl;
    std::cout << "Listening for pose on port: " << localizer_port << std::endl;
    std::cout << "Press Ctrl+C to stop recording" << std::endl;
    std::cout << std::endl;
    
    try {
        recorder = std::make_unique<LiDARRecorder>(output_dir, localizer_ip, localizer_port);
        recorder->start();
        
        // Keep running until interrupted
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } catch (const std::exception& e) {
        std::cerr << "[Main] Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
