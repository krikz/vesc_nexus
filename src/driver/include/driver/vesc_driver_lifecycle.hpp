#ifndef VESC_DRIVER_LIFECYCLE_HPP_
#define VESC_DRIVER_LIFECYCLE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "driver/vesc_packet.hpp"
#include "driver/vesc_packet_values.hpp"
#include "driver/vesc_packet_firmware_version.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"

namespace vesc_driver_lifecycle {

class VescDriverLifecycle : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit VescDriverLifecycle(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Методы управления жизненным циклом
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state);

  // Callback-функции
  void vescPacketCallback(const std::shared_ptr<vesc_driver::VescPacket const> & packet);
  void vescErrorCallback(const std::string & error);
  
  // Callback-функции для обработки команд
  void dutyCycleCallback(const std_msgs::msg::Float64::SharedPtr duty_cycle, uint8_t vesc_id);
  void currentCallback(const std_msgs::msg::Float64::SharedPtr current, uint8_t vesc_id);
  void brakeCallback(const std_msgs::msg::Float64::SharedPtr brake, uint8_t vesc_id);
  void speedCallback(const std_msgs::msg::Float64::SharedPtr speed, uint8_t vesc_id);
  void positionCallback(const std_msgs::msg::Float64::SharedPtr position, uint8_t vesc_id);
  void servoCallback(const std_msgs::msg::Float64::SharedPtr servo, uint8_t vesc_id);

  // Внутренние методы
  void timerCallback();

  // Состояния драйвера
  enum DriverMode {
    MODE_INITIALIZING,
    MODE_OPERATING,
    MODE_ACTIVE
  };

  // Структура для ограничения команд
  struct CommandLimit {
    CommandLimit(
      rclcpp_lifecycle::LifecycleNode * node_ptr,
      const std::string & str,
      const std::optional<double> & min_lower = std::nullopt,
      const std::optional<double> & max_upper = std::nullopt);

    double clip(double value);

    rclcpp_lifecycle::LifecycleNode * node_ptr;
    rclcpp::Logger logger;
    std::string name;
    std::optional<double> lower;
    std::optional<double> upper;
  };

private:
  // Члены класса
  std::vector<std::shared_ptr<vesc_driver::VescInterface>> can_interfaces_;
  std::vector<VescInfo> vescs_;
  rclcpp::TimerBase::SharedPtr timer_;
  DriverMode driver_mode_;
  int fw_version_major_;
  int fw_version_minor_;
  
  // Ограничения команд
  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;
};

}  // namespace vesc_driver_lifecycle

#endif  // VESC_DRIVER_LIFECYCLE_HPP_