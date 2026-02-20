#pragma once

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <sstream>
#include <cstring>

namespace g1_localization {

class SimpleTCPServer {
public:
    SimpleTCPServer(int port) : port_(port), running_(false) {
        setupSocket();
    }

    ~SimpleTCPServer() {
        stop();
    }

    void start() {
        running_ = true;
        accept_thread_ = std::thread(&SimpleTCPServer::acceptLoop, this);
    }

    void stop() {
        running_ = false;
        if (server_fd_ >= 0) close(server_fd_);
        if (accept_thread_.joinable()) accept_thread_.join();
    }

    // Broadcast message to all connected clients
    void broadcast(const std::string& msg) {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        auto it = clients_.begin();
        while (it != clients_.end()) {
            int sock = *it;
            // Simple length-prefix framing: [Length (4 bytes)][Data]
            uint32_t len = htonl(msg.size());
            
            ssize_t sent_len = send(sock, &len, sizeof(len), MSG_NOSIGNAL);
            ssize_t sent_data = send(sock, msg.c_str(), msg.size(), MSG_NOSIGNAL);
            
            if (sent_len < 0 || sent_data < 0) {
                // Client disconnected
                close(sock);
                it = clients_.erase(it);
            } else {
                ++it;
            }
        }
    }

private:
    void setupSocket() {
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            std::cerr << "[TCP] Failed to create socket" << std::endl;
            return;
        }

        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port_);

        if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
            std::cerr << "[TCP] Bind failed" << std::endl;
            return;
        }

        if (listen(server_fd_, 3) < 0) {
            std::cerr << "[TCP] Listen failed" << std::endl;
            return;
        }
    }

    void acceptLoop() {
        while (running_) {
            sockaddr_in address;
            socklen_t addrlen = sizeof(address);
            int new_socket = accept(server_fd_, (struct sockaddr*)&address, &addrlen);
            
            if (new_socket >= 0) {
                // Set non-blocking
                int flags = fcntl(new_socket, F_GETFL, 0);
                fcntl(new_socket, F_SETFL, flags | O_NONBLOCK);
                
                std::lock_guard<std::mutex> lock(clients_mutex_);
                clients_.push_back(new_socket);
                std::cout << "[TCP] New Client Connected!" << std::endl;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    int port_;
    int server_fd_ = -1;
    std::atomic<bool> running_;
    std::thread accept_thread_;
    std::vector<int> clients_;
    std::mutex clients_mutex_;
};

}
