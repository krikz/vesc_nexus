#include "driver/vesc_manager.hpp"

VescManager::VescManager(rclcpp::Node *node) : node_(node) {}

void VescManager::addVesc(const std::string& interface_name, uint8_t id, const std::string& label) {
    // Создаём топики с использованием логического имени
    std::string topic_prefix = "vesc/" + label;
    std::string state_topic = topic_prefix + "/state";
    std::string command_topic = topic_prefix + "/command";

    RCLCPP_INFO(rclcpp::get_logger("VescManager"), "Adding VESC: Interface=%s, ID=%d, Label=%s",
                interface_name.c_str(), id, label.c_str());

    // Пример создания публикаторов/подписчиков
    auto state_publisher = this->create_publisher<std_msgs::msg::String>(state_topic, 10);
    auto command_subscriber = this->create_subscription<std_msgs::msg::String>(
        command_topic, 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(rclcpp::get_logger("VescManager"), "Received command: %s", msg->data.c_str());
        });

    // Сохраняем интерфейс и ID
    interfaces_[interface_name].emplace_back(id, label, state_publisher, command_subscriber);
}

void VescManager::start()
{
    for (auto &pair : can_interfaces_) {
        pair.second->startListening([this](const can_frame &frame) {
            // TODO: ??????? ? ???????? ????????? VESC
        });
    }
}
