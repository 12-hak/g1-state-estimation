#pragma once

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <string>

// Simple Structure for Policy Input
struct PolicyInput {
    double position[3];
    double orientation[4]; // w, x, y, z
    double velocity[3];
    double angular_velocity[3];
    double grav_vector[3]; // New
    double q[35];
    double dq[35];
};

class StatePublisher {
public:
    StatePublisher(const std::string& name) : name_(name) {
        // Create Shared Memory
        shm_fd_ = shm_open(name_.c_str(), O_CREAT | O_RDWR, 0666);
        if (shm_fd_ == -1) {
            perror("shm_open");
            return;
        }

        // Configure size
        ftruncate(shm_fd_, sizeof(PolicyInput));

        // Map
        ptr_ = (PolicyInput*)mmap(0, sizeof(PolicyInput), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
        if (ptr_ == MAP_FAILED) {
            perror("mmap");
            return;
        }
        
        std::cout << "[StatePublisher] Shared Memory '" << name_ << "' initialized." << std::endl;
    }

    ~StatePublisher() {
        if (ptr_ != MAP_FAILED) munmap(ptr_, sizeof(PolicyInput));
        if (shm_fd_ != -1) close(shm_fd_);
        // shm_unlink(name_.c_str()); // Don't unlink if policy needs to open it!
    }

    void publish(const RobotState& state) {
        if (ptr_ == MAP_FAILED) return;
        
        // Copy data (atomic-ish if small, or use mutex/spinlock in shm for robust)
        // For 500Hz single-writer, usually fine without lock if reader checks consistency or just polls.
        // Copying...
        
        ptr_->position[0] = state.position[0];
        ptr_->position[1] = state.position[1];
        ptr_->position[2] = state.position[2];
        
        ptr_->orientation[0] = state.orientation.w();
        ptr_->orientation[1] = state.orientation.x();
        ptr_->orientation[2] = state.orientation.y();
        ptr_->orientation[3] = state.orientation.z();
        
        ptr_->velocity[0] = state.velocity[0];
        ptr_->velocity[1] = state.velocity[1];
        ptr_->velocity[2] = state.velocity[2];
        
        ptr_->angular_velocity[0] = state.angular_velocity[0];
        ptr_->angular_velocity[1] = state.angular_velocity[1];
        ptr_->angular_velocity[2] = state.angular_velocity[2];

        ptr_->grav_vector[0] = state.grav_vector[0];
        ptr_->grav_vector[1] = state.grav_vector[1];
        ptr_->grav_vector[2] = state.grav_vector[2];

        // Copy Joints
        for(int i=0; i<35; ++i) {
            ptr_->q[i] = state.q[i];
            ptr_->dq[i] = state.dq[i];
        }
    }

private:
    std::string name_;
    int shm_fd_ = -1;
    PolicyInput* ptr_ = nullptr;
};
