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


#include "vesc_driver/vesc_driver_lifecycle.hpp"
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/transition.hpp>

#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

namespace vesc_driver_lifecycle {

using namespace std::chrono_literals;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_interfaces::msg::VescStateStamped;


VescDriverLifecycle::VescDriverLifecycle(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("vesc_driver_lifecycle", options),
  vesc_(
    std::string(),
    std::bind(&VescDriverLifecycle::vescPacketCallback, this, _1),
    std::bind(&VescDriverLifecycle::vescErrorCallback, this, _1)),
  duty_cycle_limit_(this, "duty_cycle", -1.0, 1.0),
  current_limit_(this, "current"),
  brake_limit_(this, "brake"),
  speed_limit_(this, "speed"),
  position_limit_(this, "position"),
  servo_limit_(this, "servo", 0.0, 1.0),
  driver_mode_(MODE_INITIALIZING),
  fw_version_major_(-1),
  fw_version_minor_(-1)
{
  // Declare parameters here
  this->declare_parameter<std::string>("port", "/dev/ttyACM0");
  RCLCPP_INFO(get_logger(), "Node started...");
}

// ON CONFIGURE
LifecycleCallbackReturn VescDriverLifecycle::on_configure(
  const rclcpp_lifecycle::State &)  {
  RCLCPP_INFO(get_logger(), "Configuring node...");

  // Get VESC serial port address
  std::string port = get_parameter("port").as_string();
  try {
    vesc_.connect(port);
  } catch (vesc_driver::SerialException e) {
    RCLCPP_FATAL(get_logger(), "Failed to connect to the VESC: %s", e.what());
    return LifecycleCallbackReturn::FAILURE;
  }

  // Publishers
  state_pub_ = create_publisher<VescStateStamped>("sensors/core", rclcpp::QoS{10});
  servo_sensor_pub_ = create_publisher<Float64>("sensors/servo_position_command", rclcpp::QoS{10});

  // Subscriptions
  duty_cycle_sub_ = create_subscription<Float64>(
    "commands/motor/duty_cycle", rclcpp::QoS{10}, std::bind(&VescDriverLifecycle::dutyCycleCallback, this, _1));
  current_sub_ = create_subscription<Float64>(
    "commands/motor/current", rclcpp::QoS{10}, std::bind(&VescDriverLifecycle::currentCallback, this, _1));
  brake_sub_ = create_subscription<Float64>(
    "commands/motor/brake", rclcpp::QoS{10}, std::bind(&VescDriverLifecycle::brakeCallback, this, _1));
  speed_sub_ = create_subscription<Float64>(
    "commands/motor/speed", rclcpp::QoS{10}, std::bind(&VescDriverLifecycle::speedCallback, this, _1));
  position_sub_ = create_subscription<Float64>(
    "commands/motor/position", rclcpp::QoS{10}, std::bind(&VescDriverLifecycle::positionCallback, this, _1));
  servo_sub_ = create_subscription<Float64>(
    "commands/servo/position", rclcpp::QoS{10}, std::bind(&VescDriverLifecycle::servoCallback, this, _1));

  // Timer
  timer_ = this->create_wall_timer(20ms, std::bind(&VescDriverLifecycle::timerCallback, this));
  timer_->cancel();   // TODO: Run the timer until the init sequence is done before canceling

  RCLCPP_INFO(get_logger(), "Node configured successfully.");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn VescDriverLifecycle::on_activate(
  const rclcpp_lifecycle::State &)  {
  RCLCPP_INFO(get_logger(), "Activating node...");
  state_pub_->on_activate();
  servo_sensor_pub_->on_activate();
  timer_->reset();
  driver_mode_ = MODE_ACTIVE;
  RCLCPP_INFO(get_logger(), "Node activated successfully.");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn VescDriverLifecycle::on_deactivate(
  const rclcpp_lifecycle::State &)  {
  RCLCPP_INFO(get_logger(), "Deactivating node...");
  state_pub_->on_deactivate();
  timer_->cancel();
  driver_mode_ = MODE_OPERATING;
  servo_sensor_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Node deactivated successfully.");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn VescDriverLifecycle::on_cleanup(
  const rclcpp_lifecycle::State &)  {
  RCLCPP_INFO(get_logger(), "Cleaning up node...");
  state_pub_.reset();
  servo_sensor_pub_.reset();
  duty_cycle_sub_.reset();
  current_sub_.reset();
  brake_sub_.reset();
  speed_sub_.reset();
  position_sub_.reset();
  servo_sub_.reset();
  timer_->cancel();
  vesc_.disconnect();
  RCLCPP_INFO(get_logger(), "Node cleaned up successfully.");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn VescDriverLifecycle::on_shutdown(
  const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down node...");
  state_pub_.reset();
  servo_sensor_pub_.reset();
  duty_cycle_sub_.reset();
  current_sub_.reset();
  brake_sub_.reset();
  speed_sub_.reset();
  position_sub_.reset();
  servo_sub_.reset();
  
  if (timer_ && !timer_->is_canceled())
    timer_->cancel();
  
  vesc_.disconnect();
  return LifecycleCallbackReturn::SUCCESS;
}

void VescDriverLifecycle::timerCallback()
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    RCLCPP_FATAL(get_logger(), "Unexpectedly disconnected from serial port.");
    rclcpp::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      RCLCPP_INFO(
        get_logger(), "Connected to VESC with firmware version %d.%d",
        fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  } else if (driver_mode_ == MODE_OPERATING || driver_mode_ == MODE_ACTIVE) {
    // poll for vesc state (telemetry)
    vesc_.requestState();
  } else {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void VescDriverLifecycle::vescPacketCallback(const std::shared_ptr<vesc_driver::VescPacket const> & packet)
{
  if (packet->name() == "Values") {
    std::shared_ptr<vesc_driver::VescPacketValues const> values =
      std::dynamic_pointer_cast<vesc_driver::VescPacketValues const>(packet);

    auto state_msg = VescStateStamped();
    state_msg.header.stamp = now();

    state_msg.state.voltage_input = values->v_in();
    state_msg.state.current_motor = values->avg_motor_current();
    state_msg.state.current_input = values->avg_input_current();
    state_msg.state.avg_id = values->avg_id();
    state_msg.state.avg_iq = values->avg_iq();
    state_msg.state.duty_cycle = values->duty_cycle_now();
    state_msg.state.speed = values->rpm();

    state_msg.state.charge_drawn = values->amp_hours();
    state_msg.state.charge_regen = values->amp_hours_charged();
    state_msg.state.energy_drawn = values->watt_hours();
    state_msg.state.energy_regen = values->watt_hours_charged();
    state_msg.state.displacement = values->tachometer();
    state_msg.state.distance_traveled = values->tachometer_abs();
    state_msg.state.fault_code = values->fault_code();

    state_msg.state.pid_pos_now = values->pid_pos_now();
    state_msg.state.controller_id = values->controller_id();

    state_msg.state.ntc_temp_mos1 = values->temp_mos1();
    state_msg.state.ntc_temp_mos2 = values->temp_mos2();
    state_msg.state.ntc_temp_mos3 = values->temp_mos3();
    state_msg.state.avg_vd = values->avg_vd();
    state_msg.state.avg_vq = values->avg_vq();

    state_pub_->publish(state_msg);
  } else if (packet->name() == "FWVersion") {
    std::shared_ptr<vesc_driver::VescPacketFWVersion const> fw_version =
      std::dynamic_pointer_cast<vesc_driver::VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
    RCLCPP_INFO(
      get_logger(),
      "-=%s=- hardware paired %d",
      fw_version->hwname().c_str(),
      fw_version->paired()
    );
  }
  auto & clk = *this->get_clock();
  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    clk,
    5000,
    "%s packet received",
    packet->name().c_str()
  );
}

void VescDriverLifecycle::vescErrorCallback(const std::string & error)
{
  RCLCPP_ERROR(get_logger(), "%s", error.c_str());
}

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
void VescDriverLifecycle::dutyCycleCallback(const Float64::SharedPtr duty_cycle)
{
  if (driver_mode_ == MODE_ACTIVE) {
    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
  }
}

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
void VescDriverLifecycle::currentCallback(const Float64::SharedPtr current)
{
  if (driver_mode_ == MODE_ACTIVE) {
    vesc_.setCurrent(current_limit_.clip(current->data));
  }
}

/**
 * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
 *              However, note that the VESC may impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
void VescDriverLifecycle::brakeCallback(const Float64::SharedPtr brake)
{
  if (driver_mode_ == MODE_ACTIVE) {
    vesc_.setBrake(brake_limit_.clip(brake->data));
  }
}

/**
 * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
 *              multiplied by the number of motor poles. Any value is accepted by this
 *              driver. However, note that the VESC may impose a more restrictive bounds on the
 *              range depending on its configuration.
 */
void VescDriverLifecycle::speedCallback(const Float64::SharedPtr speed)
{
  if (driver_mode_ == MODE_ACTIVE) {
    vesc_.setSpeed(speed_limit_.clip(speed->data));
  }
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
void VescDriverLifecycle::positionCallback(const Float64::SharedPtr position)
{
  if (driver_mode_ == MODE_ACTIVE) {
    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    vesc_.setPosition(position_deg);
  }
}

/**
 * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
 */
void VescDriverLifecycle::servoCallback(const Float64::SharedPtr servo)
{
  if (driver_mode_ == MODE_ACTIVE) {
    double servo_clipped(servo_limit_.clip(servo->data));
    vesc_.setServo(servo_clipped);
    // publish clipped servo value as a "sensor"
    auto servo_sensor_msg = Float64();
    servo_sensor_msg.data = servo_clipped;
    servo_sensor_pub_->publish(servo_sensor_msg);
  }
}

VescDriverLifecycle::CommandLimit::CommandLimit(
  rclcpp_lifecycle::LifecycleNode * node_ptr,
  const std::string & str,
  const std::optional<double> & min_lower,
  const std::optional<double> & max_upper)
: node_ptr(node_ptr),
  logger(node_ptr->get_logger()),
  name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  auto param_min =
    node_ptr->declare_parameter(name + "_min", rclcpp::ParameterValue(0.0));

  if (param_min.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    if (min_lower && param_min.get<double>() < *min_lower) {
      lower = *min_lower;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_min (" << param_min.get<double>() <<
          ") is less than the feasible minimum (" << *min_lower << ").");
    } else if (max_upper && param_min.get<double>() > *max_upper) {
      lower = *max_upper;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_min (" << param_min.get<double>() <<
          ") is greater than the feasible maximum (" << *max_upper << ").");
    } else {
      lower = param_min.get<double>();
    }
  } else if (min_lower) {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  auto param_max =
    node_ptr->declare_parameter(name + "_max", rclcpp::ParameterValue(0.0));

  if (param_max.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    if (min_lower && param_max.get<double>() < *min_lower) {
      upper = *min_lower;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_max (" << param_max.get<double>() <<
          ") is less than the feasible minimum (" << *min_lower << ").");
    } else if (max_upper && param_max.get<double>() > *max_upper) {
      upper = *max_upper;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_max (" << param_max.get<double>() <<
          ") is greater than the feasible maximum (" << *max_upper << ").");
    } else {
      upper = param_max.get<double>();
    }
  } else if (max_upper) {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper) {
    RCLCPP_WARN_STREAM(
      logger, "Parameter " << name << "_max (" << *upper <<
        ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";

  if (lower) {
    oss << *lower << " ";
  } else {
    oss << "(none) ";
  }

  if (upper) {
    oss << *upper;
  } else {
    oss << "(none)";
  }

  RCLCPP_DEBUG_STREAM(logger, oss.str());
}

double VescDriverLifecycle::CommandLimit::clip(double value)
{
  auto clock = rclcpp::Clock(RCL_ROS_TIME);

  if (lower && value < lower) {
    RCLCPP_INFO_THROTTLE(
      logger, clock, 10, "%s command value (%f) below minimum limit (%f), clipping.",
      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper) {
    RCLCPP_INFO_THROTTLE(
      logger, clock, 10, "%s command value (%f) above maximum limit (%f), clipping.",
      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}

}  // namespace vesc_driver

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_driver_lifecycle::VescDriverLifecycle)
