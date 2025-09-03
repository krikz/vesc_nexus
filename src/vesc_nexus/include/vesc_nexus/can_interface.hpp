// can_interface.hpp
#pragma once

#include <linux/can.h>
#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>

class CanInterface {
public:
    using CanFrameCallback = std::function<void(const struct can_frame&)>;

    explicit CanInterface(const std::string& interface_name);
    ~CanInterface();

    bool open();
    void close();
    bool sendFrame(const struct can_frame& frame);
    void setReceiveCallback(CanFrameCallback cb);
    bool isRunning() const;

private:
    std::string interface_name_;
    int socket_;
    std::atomic<bool> running_;
    CanFrameCallback callback_;
    std::thread receive_thread_;
    mutable std::mutex socket_mutex_;

    void receiveLoop();
};