#ifndef TCP_SERVER_HPP
#define TCP_SERVER_HPP

#include <iostream>
#include <string>
#include <thread>
#include <functional>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

namespace color_pointscloud {

class TcpServer {
public:
    using Callback = std::function<std::string(const std::string&)>;

    TcpServer(int port) : port_(port), running_(false), server_fd_(-1) {}

    ~TcpServer() {
        stop();
    }

    void setCallback(Callback cb) {
        callback_ = cb;
    }

    bool start() {
        if (running_) return true;

        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ == -1) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }

        int opt = 1;
        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
            std::cerr << "Failed to setsockopt" << std::endl;
            close(server_fd_);
            return false;
        }

        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port_);

        if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
            std::cerr << "Failed to bind to port " << port_ << std::endl;
            close(server_fd_);
            return false;
        }

        if (listen(server_fd_, 10) < 0) {
            std::cerr << "Failed to listen on socket" << std::endl;
            close(server_fd_);
            return false;
        }

        running_ = true;
        accept_thread_ = std::thread(&TcpServer::acceptLoop, this);
        return true;
    }

    void stop() {
        if (!running_) return;
        running_ = false;
        if (server_fd_ != -1) {
            shutdown(server_fd_, SHUT_RDWR);
            close(server_fd_);
            server_fd_ = -1;
        }
        if (accept_thread_.joinable()) {
            accept_thread_.join();
        }
    }

private:
    void acceptLoop() {
        struct sockaddr_in address;
        int addrlen = sizeof(address);
        
        while (running_) {
            int new_socket = accept(server_fd_, (struct sockaddr*)&address, (socklen_t*)&addrlen);
            if (new_socket < 0) {
                if (running_) {
                    std::cerr << "Failed to accept connection" << std::endl;
                }
                continue;
            }

            handleConnection(new_socket);
        }
    }

    void handleConnection(int client_socket) {
        char buffer[1024] = {0};
        int valread = read(client_socket, buffer, sizeof(buffer) - 1);
        if (valread > 0) {
            std::string request(buffer, valread);
            std::string response = "OK";
            if (callback_) {
                response = callback_(request);
            }
            send(client_socket, response.c_str(), response.length(), 0);
        }
        // 短连接机制：单次交互结束后立即断开
        close(client_socket);
    }

    int port_;
    std::atomic<bool> running_;
    int server_fd_;
    std::thread accept_thread_;
    Callback callback_;
};

} // namespace color_pointscloud

#endif // TCP_SERVER_HPP