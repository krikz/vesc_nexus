#include "driver/vesc_driver_node.hpp"

VescNexusNode::VescNexusNode() : Node("vesc_driver_node")
{
    RCLCPP_INFO(this->get_logger(), "VescNexusNode started.");

    // Получаем все параметры
    auto parameter_names = this->list_parameters({}, 10).names;

    vesc_manager_ = std::make_unique<VescManager>(this);

    // Ищем параметры вида: can_interfaces.0.name, can_interfaces.0.baudrate, etc.
    for (const std::string &name : parameter_names)
    {
        if (name.rfind("can_interfaces.", 0) == 0 && name.find(".name") != std::string::npos)
        {
            std::string interface_name = this->get_parameter(name).as_string();
            std::string base = name.substr(0, name.find(".name"));

            rclcpp::Parameter baudrate_param, vesc_ids_param;
            if (!this->get_parameter(base + ".baudrate", baudrate_param) ||
                !this->get_parameter(base + ".vesc_ids", vesc_ids_param))
            {
                RCLCPP_WARN(this->get_logger(), "Missing baudrate or vesc_ids for %s", interface_name.c_str());
                continue;
            }
            std::string baudrate_str;
            this->get_parameter(base + ".baudrate", baudrate_str);
            int baudrate = std::stoi(baudrate_str);
            std::vector<std::string> vesc_ids_str;
            if (!this->get_parameter(base + ".vesc_ids", vesc_ids_str))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to get vesc_ids for %s", interface_name.c_str());
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Configuring CAN interface: %s (%d baud)", interface_name.c_str(), baudrate);

            for (const std::string &id_str : vesc_ids_str)
            {
                uint8_t id = static_cast<uint8_t>(std::stoi(id_str));
                vesc_manager_->addVesc(interface_name, id);
                RCLCPP_INFO(this->get_logger(), "Added VESC ID %d on %s", id, interface_name.c_str());
            }
        }
    }

    vesc_manager_->start();
}

int main(int argc, char *argv[])
{
    // Инициализация ROS 2
    RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing ROS 2...");
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "ROS 2 initialized successfully.");

    // Создание ноды
    RCLCPP_INFO(rclcpp::get_logger("main"), "Creating VescNexusNode...");
    auto node = std::make_shared<VescNexusNode>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "VescNexusNode created successfully.");

    // Запуск spin
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting rclcpp::spin...");
    try {
        rclcpp::spin(node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "rclcpp::spin completed.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception during rclcpp::spin: %s", e.what());
    }

    // Завершение работы ROS 2
    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down ROS 2...");
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("main"), "ROS 2 shutdown completed.");

    return 0;
}