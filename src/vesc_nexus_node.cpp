#include "vesc_nexus/vesc_nexus_node.hpp"

VescNexusNode::VescNexusNode() : Node("vesc_nexus_node")
{
  RCLCPP_INFO(this->get_logger(), "VescNexusNode started.");

  // Загружаем параметры
  auto declare_param = [this](const std::string &name, const auto &default_value) {
    this->declare_parameter(name, default_value);
  };

  declare_param("can_interfaces", std::vector<rclcpp::Parameter>());
  declare_param("publish_rate", 50.0);

  auto can_interfaces = this->get_parameter("can_interfaces").as_string_array();
  auto publish_rate = this->get_parameter("publish_rate").as_double();

  RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz", publish_rate);

  // Создаём менеджер
  vesc_manager_ = std::make_unique<VescManager>(this);

  // Парсим конфиг и добавляем VESC
  // Пример: ожидаем параметры в виде:
  // can_interfaces:
  //   - name: can0
  //     baudrate: "500000"
  //     vesc_ids: ["1", "2", "3", "4"]

  auto parameters = this->get_parameters("");
  for (const auto &pair : parameters) {
    const std::string &param_name = pair.first;
    if (param_name.rfind("can_interfaces.", 0) == 0) {
      // Ищем параметры вида can_interfaces.0.name, can_interfaces.0.vesc_ids и т.д.
      if (param_name.find("name") != std::string::npos) {
        std::string interface_name = pair.second.as_string();
        std::string base = param_name.substr(0, param_name.find("name") - 1); // can_interfaces.0

        auto baudrate_param = this->get_parameter(base + ".baudrate");
        auto vesc_ids_param = this->get_parameter(base + ".vesc_ids");

        if (!baudrate_param || !vesc_ids_param) {
          RCLCPP_WARN(this->get_logger(), "Missing baudrate or vesc_ids for %s", interface_name.c_str());
          continue;
        }

        int baudrate = std::stoi(baudrate_param.as_string());

        RCLCPP_INFO(this->get_logger(), "Configuring CAN interface: %s (%d baud)", interface_name.c_str(), baudrate);

        auto vesc_ids_str = vesc_ids_param.as_string_array();
        for (const std::string &id_str : vesc_ids_str) {
          uint8_t id = static_cast<uint8_t>(std::stoi(id_str));
          vesc_manager_->addVesc(interface_name, id);
          RCLCPP_INFO(this->get_logger(), "Added VESC ID %d on %s", id, interface_name.c_str());
        }
      }
    }
  }

  // Запускаем менеджер
  vesc_manager_->start();
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VescNexusNode>());
  rclcpp::shutdown();
  return 0;
}