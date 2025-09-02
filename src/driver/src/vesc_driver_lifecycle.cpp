#include "driver/vesc_driver_lifecycle.hpp"
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/transition.hpp>

#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <nlohmann/json.hpp> // Для работы с JSON

#include "driver/vesc_interface.hpp"

using json = nlohmann::json;

namespace vesc_driver_lifecycle {

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

// Структура для хранения информации о каждом VESC
struct VescInfo {
    std::string label;
    uint8_t id;
    rclcpp_lifecycle::LifecyclePublisher<VescStateStamped>::SharedPtr state_pub;
    rclcpp_lifecycle::LifecyclePublisher<Float64>::SharedPtr servo_sensor_pub;
    rclcpp::Subscription<Float64>::SharedPtr duty_cycle_sub;
    rclcpp::Subscription<Float64>::SharedPtr current_sub;
    rclcpp::Subscription<Float64>::SharedPtr brake_sub;
    rclcpp::Subscription<Float64>::SharedPtr speed_sub;
    rclcpp::Subscription<Float64>::SharedPtr position_sub;
    rclcpp::Subscription<Float64>::SharedPtr servo_sub;
};

// ИСПРАВЛЕНИЕ 1: Добавлен конструктор по умолчанию для CommandLimit
VescDriverLifecycle::CommandLimit::CommandLimit()
    : node_ptr(nullptr),
      logger(rclcpp::get_logger("CommandLimit")),
      name("unnamed") {
    RCLCPP_DEBUG_STREAM(logger, "CommandLimit default constructor called");
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
  // проверка минимального значения параметра
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

  // проверка максимального значения параметра
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

  // проверка min > max
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

VescDriverLifecycle::VescDriverLifecycle(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("vesc_driver_lifecycle", options),
  // ИСПРАВЛЕНИЕ 2: Инициализация CommandLimit с помощью параметров
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
  RCLCPP_INFO(get_logger(), "Node started...");
}

// ON CONFIGURE
LifecycleCallbackReturn VescDriverLifecycle::on_configure(
  const rclcpp_lifecycle::State &)  {
  RCLCPP_INFO(get_logger(), "Configuring node...");

  try {
    // ИСПРАВЛЕНИЕ 3: Объявляем параметр, если он еще не объявлен
    this->declare_parameter<std::string>("can_interfaces_json", R"([{"name": "can0", "baudrate": "500000", "vesc_ids": [{"id": "1", "label": "front_left"}, {"id": "2", "label": "front_right"}, {"id": "3", "label": "rear_left"}, {"id": "4", "label": "rear_right"}]}])");
    this->declare_parameter<double>("publish_rate", 100.0);

    // Загружаем JSON-строку из параметров
    std::string can_interfaces_json = get_parameter("can_interfaces_json").as_string();
    double publish_rate = get_parameter("publish_rate").as_double();

    RCLCPP_INFO(get_logger(), "Publish rate: %f", publish_rate);

    // Десериализуем JSON-строку
    auto can_interfaces = json::parse(can_interfaces_json);

    for (const auto& interface : can_interfaces) {
        std::string interface_name = interface["name"];
        int baudrate = std::stoi(interface["baudrate"].get<std::string>());

        RCLCPP_INFO(get_logger(), "Configuring CAN interface: %s (%d baud)", 
                    interface_name.c_str(), baudrate);

        // Создаем CAN-интерфейс
        auto can_interface = std::make_shared<vesc_driver::VescInterface>(
            interface_name,
            std::bind(&VescDriverLifecycle::vescPacketCallback, this, _1),
            std::bind(&VescDriverLifecycle::vescErrorCallback, this, _1)
        );

        can_interfaces_.push_back(can_interface);

        // Обрабатываем каждый VESC на этом интерфейсе
        auto vesc_ids = interface["vesc_ids"].get<std::vector<json>>();
        for (const auto& vesc_info : vesc_ids) {
            uint8_t id = static_cast<uint8_t>(std::stoi(vesc_info["id"].get<std::string>()));
            std::string label = vesc_info["label"].get<std::string>();

            RCLCPP_INFO(get_logger(), "  Adding VESC: ID=%d, Label=%s", id, label.c_str());

            // Создаем информацию о VESC
            VescInfo vesc;
            vesc.label = label;
            vesc.id = id;
            
            // Формируем префикс для топиков
            std::string topic_prefix = "vesc/" + label;
            
            // Создаем публикаторы
            vesc.state_pub = create_publisher<VescStateStamped>(
                topic_prefix + "/state", rclcpp::QoS{10});
            vesc.servo_sensor_pub = create_publisher<Float64>(
                topic_prefix + "/sensors/servo_position_command", rclcpp::QoS{10});
            
            // Создаем подписки
            vesc.duty_cycle_sub = create_subscription<Float64>(
                topic_prefix + "/commands/motor/duty_cycle", rclcpp::QoS{10},
                [this, id](const Float64::SharedPtr msg) {
                    dutyCycleCallback(msg, id);
                });
            vesc.current_sub = create_subscription<Float64>(
                topic_prefix + "/commands/motor/current", rclcpp::QoS{10},
                [this, id](const Float64::SharedPtr msg) {
                    currentCallback(msg, id);
                });
            vesc.brake_sub = create_subscription<Float64>(
                topic_prefix + "/commands/motor/brake", rclcpp::QoS{10},
                [this, id](const Float64::SharedPtr msg) {
                    brakeCallback(msg, id);
                });
            vesc.speed_sub = create_subscription<Float64>(
                topic_prefix + "/commands/motor/speed", rclcpp::QoS{10},
                [this, id](const Float64::SharedPtr msg) {
                    speedCallback(msg, id);
                });
            vesc.position_sub = create_subscription<Float64>(
                topic_prefix + "/commands/motor/position", rclcpp::QoS{10},
                [this, id](const Float64::SharedPtr msg) {
                    positionCallback(msg, id);
                });
            vesc.servo_sub = create_subscription<Float64>(
                topic_prefix + "/commands/servo/position", rclcpp::QoS{10},
                [this, id](const Float64::SharedPtr msg) {
                    servoCallback(msg, id);
                });
            
            // Добавляем VESC в список
            vescs_.push_back(vesc);
        }
    }

    // Создаем таймер для опроса состояния VESC
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
        std::bind(&VescDriverLifecycle::timerCallback, this));
    timer_->cancel();

    RCLCPP_INFO(get_logger(), "Node configured successfully.");
    return LifecycleCallbackReturn::SUCCESS;
  } catch (const std::exception &e) {
    RCLCPP_FATAL(get_logger(), "Failed to configure node: %s", e.what());
    return LifecycleCallbackReturn::FAILURE;
  }
}

LifecycleCallbackReturn VescDriverLifecycle::on_activate(
  const rclcpp_lifecycle::State &)  {
  RCLCPP_INFO(get_logger(), "Activating node...");
  
  // Активируем все публикаторы
  for (auto& vesc : vescs_) {
    vesc.state_pub->on_activate();
    vesc.servo_sensor_pub->on_activate();
  }
  
  timer_->reset();
  driver_mode_ = MODE_ACTIVE;
  RCLCPP_INFO(get_logger(), "Node activated successfully.");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn VescDriverLifecycle::on_deactivate(
  const rclcpp_lifecycle::State &)  {
  RCLCPP_INFO(get_logger(), "Deactivating node...");
  
  timer_->cancel();
  driver_mode_ = MODE_OPERATING;
  
  // Деактивируем все публикаторы
  for (auto& vesc : vescs_) {
    vesc.state_pub->on_deactivate();
    vesc.servo_sensor_pub->on_deactivate();
  }
  
  RCLCPP_INFO(get_logger(), "Node deactivated successfully.");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn VescDriverLifecycle::on_cleanup(
  const rclcpp_lifecycle::State &)  {
  RCLCPP_INFO(get_logger(), "Cleaning up node...");
  
  // Сбрасываем все публикаторы и подписки
  for (auto& vesc : vescs_) {
    vesc.state_pub.reset();
    vesc.servo_sensor_pub.reset();
    vesc.duty_cycle_sub.reset();
    vesc.current_sub.reset();
    vesc.brake_sub.reset();
    vesc.speed_sub.reset();
    vesc.position_sub.reset();
    vesc.servo_sub.reset();
  }
  vescs_.clear();
  
  can_interfaces_.clear();
  
  if (timer_ && !timer_->is_canceled()) {
    timer_->cancel();
  }
  
  RCLCPP_INFO(get_logger(), "Node cleaned up successfully.");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn VescDriverLifecycle::on_shutdown(
  const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down node...");
  
  // Сбрасываем все публикаторы и подписки
  for (auto& vesc : vescs_) {
    vesc.state_pub.reset();
    vesc.servo_sensor_pub.reset();
    vesc.duty_cycle_sub.reset();
    vesc.current_sub.reset();
    vesc.brake_sub.reset();
    vesc.speed_sub.reset();
    vesc.position_sub.reset();
    vesc.servo_sub.reset();
  }
  vescs_.clear();
  
  can_interfaces_.clear();
  
  if (timer_ && !timer_->is_canceled()) {
    timer_->cancel();
  }
  
  return LifecycleCallbackReturn::SUCCESS;
}

void VescDriverLifecycle::timerCallback()
{
  // Проверяем подключение ко всем CAN-интерфейсам
  for (size_t i = 0; i < can_interfaces_.size(); ++i) {
    if (!can_interfaces_[i]->isConnected()) {
      RCLCPP_ERROR(get_logger(), "Disconnected from CAN interface %zu", i);
      // Можно добавить логику переподключения здесь
    }
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    // Запрашиваем версию прошивки для всех VESC
    for (size_t i = 0; i < vescs_.size(); ++i) {
      can_interfaces_[0]->requestFWVersion(vescs_[i].id);
    }
    
    // В реальной реализации нужно проверить получение версии для всех VESC
    // Для простоты просто устанавливаем флаг после первого запроса
    driver_mode_ = MODE_OPERATING;
  } else if (driver_mode_ == MODE_OPERATING || driver_mode_ == MODE_ACTIVE) {
    // Опрашиваем состояние всех VESC
    for (size_t i = 0; i < vescs_.size(); ++i) {
      can_interfaces_[0]->requestState(vescs_[i].id);
    }
  } else {
    // неизвестный режим
    RCLCPP_ERROR(get_logger(), "Unknown driver mode: %d", driver_mode_);
  }
}

// ИСПРАВЛЕНИЕ 4: Убран дополнительный аргумент из сигнатуры
void VescDriverLifecycle::vescPacketCallback(
  const std::shared_ptr<vesc_driver::VescPacket const> & packet)
{
  // Определяем ID VESC из пакета
  uint8_t vesc_id = packet->vesc_id();
  
  // Находим соответствующий VESC в списке
  auto it = std::find_if(vescs_.begin(), vescs_.end(), 
    [vesc_id](const VescInfo& vesc) { return vesc.id == vesc_id; });
  
  if (it == vescs_.end()) {
    RCLCPP_WARN(get_logger(), "Received packet for unknown VESC ID: %d", vesc_id);
    return;
  }
  
  VescInfo& vesc = *it;
  
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

    vesc.state_pub->publish(state_msg);
  } else if (packet->name() == "FWVersion") {
    std::shared_ptr<vesc_driver::VescPacketFWVersion const> fw_version =
      std::dynamic_pointer_cast<vesc_driver::VescPacketFWVersion const>(packet);
    
    RCLCPP_INFO(
      get_logger(),
      "VESC %s (ID: %d) firmware version: %d.%d, hardware: %s, paired: %d",
      vesc.label.c_str(), vesc.id,
      fw_version->fwMajor(), fw_version->fwMinor(),
      fw_version->hwname().c_str(), fw_version->paired()
    );
  }
  
  auto & clk = *this->get_clock();
  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    clk,
    5000,
    "VESC %s (ID: %d) - %s packet received",
    vesc.label.c_str(), vesc.id, packet->name().c_str()
  );
}

// ИСПРАВЛЕНИЕ 5: Убран дополнительный аргумент из сигнатуры
void VescDriverLifecycle::vescErrorCallback(const std::string & error)
{
  RCLCPP_ERROR(get_logger(), "CAN interface error: %s", error.c_str());
}

// ИСПРАВЛЕНИЕ 6: Добавлены методы с правильными сигнатурами
void VescDriverLifecycle::dutyCycleCallback(const Float64::SharedPtr duty_cycle, uint8_t vesc_id)
{
  if (driver_mode_ == MODE_ACTIVE) {
    for (auto& can_interface : can_interfaces_) {
      can_interface->setDutyCycle(vesc_id, duty_cycle_limit_.clip(duty_cycle->data));
    }
  }
}

void VescDriverLifecycle::currentCallback(const Float64::SharedPtr current, uint8_t vesc_id)
{
  if (driver_mode_ == MODE_ACTIVE) {
    for (auto& can_interface : can_interfaces_) {
      can_interface->setCurrent(vesc_id, current_limit_.clip(current->data));
    }
  }
}

void VescDriverLifecycle::brakeCallback(const Float64::SharedPtr brake, uint8_t vesc_id)
{
  if (driver_mode_ == MODE_ACTIVE) {
    for (auto& can_interface : can_interfaces_) {
      can_interface->setBrake(vesc_id, brake_limit_.clip(brake->data));
    }
  }
}

void VescDriverLifecycle::speedCallback(const Float64::SharedPtr speed, uint8_t vesc_id)
{
  if (driver_mode_ == MODE_ACTIVE) {
    for (auto& can_interface : can_interfaces_) {
      can_interface->setSpeed(vesc_id, speed_limit_.clip(speed->data));
    }
  }
}

void VescDriverLifecycle::positionCallback(const Float64::SharedPtr position, uint8_t vesc_id)
{
  if (driver_mode_ == MODE_ACTIVE) {
    // ROS использует радианы, VESC использует градусы. Конвертируем.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    for (auto& can_interface : can_interfaces_) {
      can_interface->setPosition(vesc_id, position_deg);
    }
  }
}

void VescDriverLifecycle::servoCallback(const Float64::SharedPtr servo, uint8_t vesc_id)
{
  if (driver_mode_ == MODE_ACTIVE) {
    for (auto& can_interface : can_interfaces_) {
      double servo_clipped = servo_limit_.clip(servo->data);
      can_interface->setServo(vesc_id, servo_clipped);
      
      // Публикуем значение серво как "сенсор"
      auto it = std::find_if(vescs_.begin(), vescs_.end(), 
        [vesc_id](const VescInfo& vesc) { return vesc.id == vesc_id; });
      
      if (it != vescs_.end()) {
        auto servo_sensor_msg = Float64();
        servo_sensor_msg.data = servo_clipped;
        it->servo_sensor_pub->publish(servo_sensor_msg);
      }
    }
  }
}

}  // namespace vesc_driver_lifecycle

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_driver_lifecycle::VescDriverLifecycle)