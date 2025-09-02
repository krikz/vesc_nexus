#include "driver/vesc_driver_node.hpp"
#include <stdexcept>
#include <nlohmann/json.hpp> // Библиотека для работы с JSON

// Удобный алиас для JSON
using json = nlohmann::json;

VescNexusNode::VescNexusNode(const rclcpp::NodeOptions& options)
    : Node("vesc_driver_node", options)
{
    RCLCPP_INFO(this->get_logger(), "VescNexusNode started.");

    // Объявляем параметры
    this->declare_parameter<std::string>("can_interfaces_json", "[]");
    this->declare_parameter<double>("publish_rate", 100.0);

    // Инициализируем менеджер VESC
    vesc_manager_ = std::make_unique<VescManager>(this);

    try {
        // Читаем JSON-строку из параметров
        std::string can_interfaces_json = this->get_parameter("can_interfaces_json").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "Publish rate: %f", publish_rate);

        // Десериализуем JSON-строку
        auto can_interfaces = json::parse(can_interfaces_json);

        for (const auto& interface : can_interfaces) {
            std::string interface_name = interface["name"];
            int baudrate = std::stoi(interface["baudrate"].get<std::string>());

            auto vesc_ids = interface["vesc_ids"].get<std::vector<json>>();
            for (const auto& vesc_info : vesc_ids) {
                uint8_t id = static_cast<uint8_t>(std::stoi(vesc_info["id"].get<std::string>()));
                std::string label = vesc_info["label"].get<std::string>();

                RCLCPP_INFO(this->get_logger(), "Configuring CAN interface: %s (%d baud), VESC ID: %d, Label: %s",
                            interface_name.c_str(), baudrate, id, label.c_str());

                // Добавляем VESC в менеджер
                vesc_manager_->addVesc(interface_name, id, label);
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load parameters: %s", e.what());
        throw;
    }

    // Запускаем менеджер VESC
    vesc_manager_->start();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(VescNexusNode)