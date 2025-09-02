#include "driver/can_interface.hpp"
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <unistd.h>
#include <iostream>
#include <cstring>

CanInterface::CanInterface(const std::string &interface_name) : interface_name_(interface_name), socket_(-1)
{
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
        throw std::runtime_error("Failed to create CAN socket");
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface_name.c_str());
    ioctl(socket_, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(socket_);
        throw std::runtime_error("Failed to bind CAN socket");
    }
}

void CanInterface::startListening(std::function<void(const can_frame &)> callback)
{
    std::thread([this, callback]() {
        can_frame frame;
        while (running_) {
            int nbytes = read(socket_, &frame, sizeof(frame));
            if (nbytes > 0) {
                callback(frame);
            }
        }
    }).detach();
}

void CanInterface::sendFrame(const can_frame &frame)
{
    write(socket_, &frame, sizeof(frame));
}