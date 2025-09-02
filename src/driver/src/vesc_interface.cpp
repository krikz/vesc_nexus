#include "driver/vesc_interface.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <cstring>    // Для strncpy
#include <unistd.h>   // Для close, read, write
#include <sys/types.h>
#include <sys/socket.h>

#include "driver/vesc_packet_factory.hpp"

namespace vesc_driver {

class VescInterface::Impl {
public:
    Impl()
        : can_socket_(-1), packet_thread_run_(false) {}

    void packet_creation_thread();
    void connect(const std::string &can_interface);
    void disconnect();

    bool packet_thread_run_;
    std::unique_ptr<std::thread> packet_thread_;
    PacketHandlerFunction packet_handler_;
    ErrorHandlerFunction error_handler_;
    int can_socket_;

    ~Impl() {
        if (can_socket_ >= 0) {
            close(can_socket_);
        }
    }

private:
    std::vector<uint8_t> buffer_;
};

void VescInterface::Impl::connect(const std::string &can_interface) {
    // Создание сокета CAN
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        throw std::runtime_error("Failed to create CAN socket");
    }

    // Получение индекса интерфейса
  struct ifreq ifr;
  strncpy(ifr.ifr_name, can_interface.c_str(), IFNAMSIZ);
  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      int error = errno;
      close(can_socket_);
      throw std::runtime_error("Failed to get CAN interface index for " + can_interface + 
                            ": " + strerror(error));
  }

    // Привязка сокета к интерфейсу
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(can_socket_);
        throw std::runtime_error("Failed to bind CAN socket");
    }

    std::cout << "Connected to CAN interface: " << can_interface << std::endl;
}

void VescInterface::Impl::disconnect() {
    if (can_socket_ >= 0) {
        close(can_socket_);
        can_socket_ = -1;
    }

    if (packet_thread_run_) {
        packet_thread_run_ = false;
        if (packet_thread_ && packet_thread_->joinable()) {
            packet_thread_->join();
        }
    }
}

void VescInterface::Impl::packet_creation_thread() {
    while (packet_thread_run_) {
        struct can_frame frame;
        int bytes_read = read(can_socket_, &frame, sizeof(struct can_frame));
        if (bytes_read > 0) {
            // Извлекаем ID сообщения (удаляем флаги)
            uint32_t can_id = frame.can_id & CAN_EFF_MASK;
            
            // Проверяем, является ли это сообщение от VESC
            if (can_id >= 0x00 && can_id <= 0xFF) {
                // Преобразуем CAN-фрейм в буфер для обработки
                std::vector<uint8_t> buffer;
                buffer.reserve(frame.can_dlc + 2); // +2 для префикса
                
                // Добавляем префикс, как в UART (SOF и длина)
                buffer.push_back(VescFrame::VESC_SOF_VAL_SMALL_FRAME);
                buffer.push_back(frame.can_dlc);
                
                // Добавляем данные из CAN-фрейма
                buffer.insert(buffer.end(), frame.data, frame.data + frame.can_dlc);
                
                // Теперь используем существующий метод createPacket
                int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
                std::string error;
                auto iter = buffer.begin();
                VescPacketConstPtr packet = VescPacketFactory::createPacket(
                    iter, buffer.end(), &bytes_needed, &error);
                
                if (packet) {
                    packet_handler_(packet);
                } else if (!error.empty()) {
                    error_handler_(error);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

VescInterface::VescInterface(
    const std::string &can_interface,
    const PacketHandlerFunction &packet_handler,
    const ErrorHandlerFunction &error_handler)
    : impl_(new Impl()) {
    setPacketHandler(packet_handler);
    setErrorHandler(error_handler);
    connect(can_interface);
}

VescInterface::~VescInterface() {
    disconnect();
}

void VescInterface::setPacketHandler(const PacketHandlerFunction &handler) {
    impl_->packet_handler_ = handler;
}

void VescInterface::setErrorHandler(const ErrorHandlerFunction &handler) {
    impl_->error_handler_ = handler;
}

void VescInterface::connect(const std::string &can_interface) {
    impl_->connect(can_interface);

    // Запуск потока для обработки пакетов
    impl_->packet_thread_run_ = true;
    impl_->packet_thread_ = std::make_unique<std::thread>(
        &VescInterface::Impl::packet_creation_thread, impl_.get());
}

void VescInterface::disconnect() {
    impl_->disconnect();
}

bool VescInterface::isConnected() const {
    return impl_->can_socket_ >= 0;
}

void VescInterface::send(const VescPacket &packet) {
    // Получаем данные пакета
    const auto& frame_data = packet.frame();
    
    // Создаем CAN-фрейм
    struct can_frame can_frame;
    std::memset(&can_frame, 0, sizeof(can_frame));
    
    // Устанавливаем CAN ID (0x00 для команд)
    can_frame.can_id = 0x00;
    
    // Копируем данные (пропуская префикс UART)
    // В UART-пакете первые 2 байта - SOF и длина
    size_t data_length = std::min<size_t>(frame_data.size() - 2, 8);
    std::memcpy(can_frame.data, frame_data.data() + 2, data_length);
    can_frame.can_dlc = data_length;
    
    // Отправляем CAN-фрейм
    if (write(impl_->can_socket_, &can_frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        throw std::runtime_error("Failed to send CAN frame");
    }
}

void VescInterface::requestFWVersion() {
    send(VescPacketRequestFWVersion());
}

void VescInterface::requestState() {
    send(VescPacketRequestValues());
}

void VescInterface::setDutyCycle(double duty_cycle) {
    send(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setCurrent(double current) {
    send(VescPacketSetCurrent(current));
}

void VescInterface::setBrake(double brake) {
    send(VescPacketSetCurrentBrake(brake));
}

void VescInterface::setSpeed(double speed) {
    send(VescPacketSetRPM(speed));
}

void VescInterface::setPosition(double position) {
    send(VescPacketSetPos(position));
}

void VescInterface::setServo(double servo) {
    send(VescPacketSetServoPos(servo));
}

}  // namespace vesc_driver