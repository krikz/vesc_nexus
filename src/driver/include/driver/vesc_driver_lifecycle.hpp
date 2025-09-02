// Copyright 2020 F1TENTH Foundation
// Copyright 2024 James Petri, Maynooth University, AURA Project
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice, this list of
//     conditions, and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice, this list of
//     conditions, and the following disclaimer in the documentation and/or other materials
//     provided with the distribution.
//
//   * Neither the name of the F1TENTH Foundation, James Petri, Maynooth University, AURA Project,
//     nor the names of its contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// -*- mode:c++; fill-column: 100; -*-
//
// This code is an open-source modification of the original VESC code by the F1TENTH Foundation,
// adapted to work as a lifecycle node. The modifications were made by James Petri, Maynooth
// University, as part of the AURA Project in 2024. The code is open source and free to use and
// modify. The author provides no guarantee of bug fixes, updates, or continued support.

#ifndef VESC_DRIVER__VESC_DRIVER_LIFECYCLE_HPP_
#define VESC_DRIVER__VESC_DRIVER_LIFECYCLE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_interfaces/msg/vesc_state.hpp>
#include <vesc_interfaces/msg/vesc_state_stamped.hpp>
#include <optional>
#include <memory>
#include <string>

#include "vesc_driver/vesc_interface.hpp"
#include "vesc_driver/vesc_packet.hpp"

namespace vesc_driver_lifecycle
{

using std_msgs::msg::Float64;
using vesc_interfaces::msg::VescState;
using vesc_interfaces::msg::VescStateStamped;
using LifecycleCallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class VescDriverLifecycle : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit VescDriverLifecycle(const rclcpp::NodeOptions & options);

  // Lifecycle node interface callbacks
  LifecycleCallbackReturn on_configure (const rclcpp_lifecycle::State & prev_state) override;
  LifecycleCallbackReturn on_activate  (const rclcpp_lifecycle::State & prev_state) override;
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State & prev_state) override;
  LifecycleCallbackReturn on_cleanup   (const rclcpp_lifecycle::State & prev_state) override;
  LifecycleCallbackReturn on_shutdown  (const rclcpp_lifecycle::State & prev_state) override;

private:
  // Interface to the VESC
  vesc_driver::VescInterface vesc_;
  void vescPacketCallback(const std::shared_ptr<vesc_driver::VescPacket const> & packet);
  void vescErrorCallback(const std::string & error);

  // Limits on VESC commands
  struct CommandLimit
  {
    CommandLimit(
      rclcpp_lifecycle::LifecycleNode * node_ptr,
      const std::string & str,
      const std::optional<double> & min_lower = std::optional<double>(),
      const std::optional<double> & max_upper =
        std::optional<double>());
    double clip(double value);
    rclcpp_lifecycle::LifecycleNode * node_ptr;
    rclcpp::Logger logger;
    std::string name;
    std::optional<double> lower;
    std::optional<double> upper;

  };

  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // ROS services and publishers/subscribers
  rclcpp_lifecycle::LifecyclePublisher<VescStateStamped>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<Float64>::SharedPtr servo_sensor_pub_;
  rclcpp::SubscriptionBase::SharedPtr duty_cycle_sub_;
  rclcpp::SubscriptionBase::SharedPtr current_sub_;
  rclcpp::SubscriptionBase::SharedPtr brake_sub_;
  rclcpp::SubscriptionBase::SharedPtr speed_sub_;
  rclcpp::SubscriptionBase::SharedPtr position_sub_;
  rclcpp::SubscriptionBase::SharedPtr servo_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Driver modes (possible states)
  typedef enum
  {
    MODE_INITIALIZING,
    MODE_OPERATING,
    MODE_ACTIVE
  }
  driver_mode_t;

  // Other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by vesc
  int fw_version_minor_;                ///< firmware minor version reported by vesc

  // ROS callbacks
  void brakeCallback(const Float64::SharedPtr brake);
  void currentCallback(const Float64::SharedPtr current);
  void dutyCycleCallback(const Float64::SharedPtr duty_cycle);
  void positionCallback(const Float64::SharedPtr position);
  void servoCallback(const Float64::SharedPtr servo);
  void speedCallback(const Float64::SharedPtr speed);
  void timerCallback();
};


}  // namespace vesc_driver_lifecycle

#endif  // VESC_DRIVER__VESC_DRIVER_LIFECYCLE_HPP_
