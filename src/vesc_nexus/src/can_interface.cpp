// can_interface.cpp
#include "vesc_nexus/can_interface.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <cerrno>

CanInterface::CanInterface(const std::string& interface_name)
    : interface_name_(interface_name), socket_(-1), running_(false) {}

CanInterface::~CanInterface() {
    close();
}

bool CanInterface::open() {
    if (running_) return true;

    struct sockaddr_can addr;
    struct ifreq ifr;

    socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("CanInterface"), "Failed to create CAN socket");
        return false;
    }

    std::strcpy(ifr.ifr_name, interface_name_.c_str());
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("CanInterface"), "Cannot find CAN interface: %s", interface_name_.c_str());
        ::close(socket_);
        socket_ = -1;
        return false;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("CanInterface"), "Cannot bind to CAN socket");
        ::close(socket_);
        socket_ = -1;
        return false;
    }

    running_ = true;
    receive_thread_ = std::thread(&CanInterface::receiveLoop, this);

    RCLCPP_INFO(rclcpp::get_logger("CanInterface"), "CAN interface %s opened successfully", interface_name_.c_str());
    return true;
}

void CanInterface::close() {
    if (running_) {
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
        if (socket_ >= 0) {
            ::close(socket_);
            socket_ = -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("CanInterface"), "CAN interface closed");
    }
}

bool CanInterface::sendFrame(const struct can_frame& frame) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (!running_ || socket_ < 0) return false;

    int nbytes = ::write(socket_, &frame, sizeof(struct can_frame));
    return (nbytes == sizeof(struct can_frame));
}

void CanInterface::setReceiveCallback(CanFrameCallback cb) {
    callback_ = cb;
}

bool CanInterface::isRunning() const {
    return running_;
}

void CanInterface::receiveLoop() {
    struct can_frame frame;
    while (running_) {
        int nbytes = ::read(socket_, &frame, sizeof(struct can_frame));
        if (nbytes > 0 && callback_) {
            callback_(frame);
        } else if (nbytes < 0) {
            if (errno != EAGAIN) {
                RCLCPP_ERROR(rclcpp::get_logger("CanInterface"), "Error reading CAN socket");
                break;
            }
        }
    }
}