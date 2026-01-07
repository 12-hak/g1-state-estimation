#include "UnitreeInterface.hpp"
#include <iostream>
#include <unitree/robot/channel/channel_factory.hpp>

// Constants
static const std::string HG_STATE_TOPIC = "rt/lowstate";

UnitreeInterface::UnitreeInterface(const std::string& network_interface) {
    // Initialize ChannelFactory (Global singleton in SDK)
    // Note: If multiple interfaces are instantiated, this might need care.
    // Assuming single instance for now.
    ChannelFactory::Instance()->Init(0, network_interface);

    // Create Subscriber
    lowstate_sub_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_sub_->InitChannel(std::bind(&UnitreeInterface::LowStateHandler, this, std::placeholders::_1), 1);
    
    std::cout << "[UnitreeInterface] Initialized on " << network_interface << std::endl;
}

UnitreeInterface::~UnitreeInterface() {
    // Cleanup if necessary
}

void UnitreeInterface::LowStateHandler(const void* message) {
    const LowState_* low_state = (const LowState_*)message;

    // Lock and copy data
    std::lock_guard<std::mutex> lock(data_mutex_);

    // IMU
    auto& imu = low_state->imu_state();
    latest_data_.quaternion[0] = imu.quaternion()[0]; // w
    latest_data_.quaternion[1] = imu.quaternion()[1]; // x
    latest_data_.quaternion[2] = imu.quaternion()[2]; // y
    latest_data_.quaternion[3] = imu.quaternion()[3]; // z

    latest_data_.gyroscope[0] = imu.gyroscope()[0];
    latest_data_.gyroscope[1] = imu.gyroscope()[1];
    latest_data_.gyroscope[2] = imu.gyroscope()[2];

    latest_data_.accelerometer[0] = imu.accelerometer()[0];
    latest_data_.accelerometer[1] = imu.accelerometer()[1];
    latest_data_.accelerometer[2] = imu.accelerometer()[2];

    // Joints
    auto& motors = low_state->motor_state();
    int num_motors = motors.size();
    for(int i=0; i < num_motors && i < 35; ++i) {
        latest_data_.q[i] = motors[i].q();
        latest_data_.dq[i] = motors[i].dq();
        latest_data_.tau[i] = motors[i].tau_est();
    }
}

G1SensorData UnitreeInterface::get_latest_data() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_data_;
}
