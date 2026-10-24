#include "vesc_nexus/can_interface.hpp"

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace vesc_nexus {
namespace {

rclcpp::Logger logger() { return rclcpp::get_logger("CanInterface"); }

rclcpp::Clock& throttleClock() {
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

/// Таймаут read(): поток приёма периодически просыпается и замечает close().
constexpr long kReceiveTimeoutUs = 100000;  // 100 мс

}  // namespace

CanInterface::CanInterface(std::string interface_name) : interface_name_(std::move(interface_name)) {}

CanInterface::~CanInterface() { close(); }

void CanInterface::setReceiveCallback(FrameCallback callback) { callback_ = std::move(callback); }

bool CanInterface::open() {
  if (running_) return true;

  socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    RCLCPP_ERROR(logger(), "socket(PF_CAN, SOCK_RAW): %s", std::strerror(errno));
    return false;
  }

  ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(logger(), "CAN interface '%s' not found: %s", interface_name_.c_str(), std::strerror(errno));
    ::close(socket_);
    socket_ = -1;
    return false;
  }

  timeval timeout{};
  timeout.tv_usec = kReceiveTimeoutUs;
  if (::setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    RCLCPP_WARN(logger(), "setsockopt(SO_RCVTIMEO) failed: %s — close() may block until the next frame",
                std::strerror(errno));
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(logger(), "bind('%s'): %s", interface_name_.c_str(), std::strerror(errno));
    ::close(socket_);
    socket_ = -1;
    return false;
  }

  running_ = true;
  receive_thread_ = std::thread(&CanInterface::receiveLoop, this);
  RCLCPP_INFO(logger(), "CAN interface '%s' opened", interface_name_.c_str());
  return true;
}

void CanInterface::close() {
  if (!running_ && socket_ < 0) return;

  running_ = false;
  if (receive_thread_.joinable()) receive_thread_.join();

  std::lock_guard<std::mutex> lock(send_mutex_);
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
  RCLCPP_INFO(logger(), "CAN interface '%s' closed", interface_name_.c_str());
}

bool CanInterface::sendFrame(const can_frame& frame) {
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (!running_ || socket_ < 0) {
    RCLCPP_WARN_THROTTLE(logger(), throttleClock(), 5000, "sendFrame: interface '%s' is not open",
                         interface_name_.c_str());
    return false;
  }

  const ssize_t written = ::write(socket_, &frame, sizeof(frame));
  if (written != static_cast<ssize_t>(sizeof(frame))) {
    RCLCPP_WARN_THROTTLE(logger(), throttleClock(), 5000, "write('%s') failed: %s", interface_name_.c_str(),
                         std::strerror(errno));
    return false;
  }
  return true;
}

void CanInterface::receiveLoop() {
  can_frame frame{};
  while (running_) {
    const ssize_t received = ::read(socket_, &frame, sizeof(frame));
    if (received == static_cast<ssize_t>(sizeof(frame))) {
      if (callback_) callback_(frame);
    } else if (received < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
      RCLCPP_ERROR(logger(), "read('%s') failed: %s — receive loop stopped", interface_name_.c_str(),
                   std::strerror(errno));
      break;
    }
  }
}

}  // namespace vesc_nexus
