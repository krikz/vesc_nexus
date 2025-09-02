#ifndef VESC_INTERFACE_HPP_
#define VESC_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include "driver/vesc_packet_factory.hpp"

namespace vesc_driver {

class VescInterface {
public:
    using PacketHandlerFunction = std::function<void(const VescPacketConstPtr &)>;
    using ErrorHandlerFunction = std::function<void(const std::string &)>;

    VescInterface(
        const std::string &can_interface,
        const PacketHandlerFunction &packet_handler,
        const ErrorHandlerFunction &error_handler);

    ~VescInterface();

    void setPacketHandler(const PacketHandlerFunction &handler);
    void setErrorHandler(const ErrorHandlerFunction &handler);

    void connect(const std::string &can_interface);
    void disconnect();
    bool isConnected() const;

    void send(const VescPacket &packet);

    void requestFWVersion();
    void requestState();
    void setDutyCycle(double duty_cycle);
    void setCurrent(double current);
    void setBrake(double brake);
    void setSpeed(double speed);
    void setPosition(double position);
    void setServo(double servo);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace vesc_driver

#endif  // VESC_INTERFACE_HPP_