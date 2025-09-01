#ifndef CAN_INTERFACE_HPP_
#define CAN_INTERFACE_HPP_

#include <linux/can.h>
#include <functional>
#include <string>
#include <thread>
#include <sys/ioctl.h>
#include <net/if.h>

class CanInterface
{
public:
  explicit CanInterface(const std::string &interface_name);
  void startListening(std::function<void(const can_frame&)> callback);
  void sendFrame(const can_frame &frame);

private:
  std::string interface_name_;
  int socket_;
  std::thread listener_thread_;
  bool running_ = false;
};

#endif // CAN_INTERFACE_HPP_
