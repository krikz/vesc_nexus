#include "driver/vesc_driver_node.hpp"
#include <stdexcept>

VescNexusNode::VescNexusNode(const rclcpp::NodeOptions& options)
    : Node("vesc_driver_node", modifyNodeOptions(options))
{
    RCLCPP_INFO(this->get_logger(), "VescNexusNode started.");

    // Инициализируем менеджер VESC
    vesc_manager_ = std::make_unique<VescManager>(this);

    // Читаем параметры из YAML-файла
    try {
        // Получаем массив интерфейсов
        std::map<std::string, rclcpp::Parameter> can_interfaces;
        this->get_parameters("can_interfaces", can_interfaces);

        //auto can_interfaces = this->get_parameter("can_interfaces").as_string_array();

        for (size_t i = 0; i < can_interfaces.size(); ++i) {
            std::string base_key = "can_interfaces." + std::to_string(i);

            // Читаем параметры для каждого интерфейса
            std::string interface_name = this->get_parameter(base_key + ".name").as_string();
            std::string baudrate_str = this->get_parameter(base_key + ".baudrate").as_string();
            int baudrate = std::stoi(baudrate_str);

            auto vesc_ids_str = this->get_parameter(base_key + ".vesc_ids").as_string_array();
            std::vector<uint8_t> vesc_ids;
            for (const auto& id_str : vesc_ids_str) {
                vesc_ids.push_back(static_cast<uint8_t>(std::stoi(id_str)));
            }

            RCLCPP_INFO(this->get_logger(), "Configuring CAN interface: %s (%d baud)", interface_name.c_str(), baudrate);

            // Добавляем VESC в менеджер
            for (uint8_t id : vesc_ids) {
                vesc_manager_->addVesc(interface_name, id);
                RCLCPP_INFO(this->get_logger(), "Added VESC ID %d on %s", id, interface_name.c_str());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load parameters: %s", e.what());
        throw;
    }

    // Запускаем менеджер VESC
    vesc_manager_->start();
}

// Вспомогательная функция для модификации NodeOptions
rclcpp::NodeOptions VescNexusNode::modifyNodeOptions(const rclcpp::NodeOptions& options) {
    rclcpp::NodeOptions modified_options = options;
    modified_options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    return modified_options;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(VescNexusNode)