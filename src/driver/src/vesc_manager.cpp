#include "driver/vesc_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Конструктор
VescManager::VescManager(rclcpp::Node* node) : node_(node) {}

// Метод для добавления VESC
void VescManager::addVesc(const std::string& interface_name, uint8_t id, const std::string& label) {
    // Формируем префикс для топиков
    std::string topic_prefix = "vesc/" + label;
    std::string state_topic = topic_prefix + "/state";
    std::string command_topic = topic_prefix + "/command";

    // Логирование
    RCLCPP_INFO(node_->get_logger(), "Adding VESC: Interface=%s, ID=%d, Label=%s",
                interface_name.c_str(), id, label.c_str());

    // Создание публикатора
    auto state_publisher = node_->create_publisher<std_msgs::msg::String>(state_topic, 10);

    // Создание подписчика
    auto command_subscriber = node_->create_subscription<std_msgs::msg::String>(
        command_topic, 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "Received command: %s", msg->data.c_str());
        });

    // Сохраняем информацию о VESC
    interfaces_[interface_name].push_back(VescInfo{id, label, state_publisher, command_subscriber});
}