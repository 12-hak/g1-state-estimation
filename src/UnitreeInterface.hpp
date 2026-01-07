#pragma once

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <array>
#include <vector>

// Unitree SDK2 Includes
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

struct G1SensorData {
    // IMU (Body Frame)
    std::array<float, 4> quaternion; // w, x, y, z
    std::array<float, 3> gyroscope;
    std::array<float, 3> accelerometer;

    // Joints (Ordered 0-29 as per SDK)
    std::array<float, 35> q;      // 35 to be safe (G1 has ~29-30)
    std::array<float, 35> dq;
    std::array<float, 35> tau;
};

class UnitreeInterface {
public:
    UnitreeInterface(const std::string& network_interface);
    ~UnitreeInterface();

    G1SensorData get_latest_data();

private:
    void LowStateHandler(const void* message);
    
    // SDK Components
    ChannelSubscriberPtr<LowState_> lowstate_sub_;
    
    // Data Storage
    G1SensorData latest_data_;
    std::mutex data_mutex_;
};
