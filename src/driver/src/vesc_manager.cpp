#include "driver/vesc_manager.hpp"
#include "vesc_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" // Включение типа String

class VescManager {
public:
    explicit VescManager(rclcpp::Node* node) : node_(node) {}

    void addVesc(const std::string& interface_name, uint8_t id, const std::string& label) {
        std::string topic_prefix = "vesc/" + label;
        std::string state_topic = topic_prefix + "/state";
        std::string command_topic = topic_prefix + "/command";

        RCLCPP_INFO(rclcpp::get_logger("VescManager"), "Adding VESC: Interface=%s, ID=%d, Label=%s",
                    interface_name.c_str(), id, label.c_str());

        // Создание публикатора
        auto state_publisher = node_->create_publisher<std_msgs::msg::String>(state_topic, 10);

        // Создание подписчика
        auto command_subscriber = node_->create_subscription<std_msgs::msg::String>(
            command_topic, 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(rclcpp::get_logger("VescManager"), "Received command: %s", msg->data.c_str());
            });

        // Сохранение интерфейса и ID
        interfaces_[interface_name].emplace_back(id, label, state_publisher, command_subscriber);
    }

private:
    rclcpp::Node* node_; // Указатель на ноду
    std::map<std::string, std::vector<std::tuple<uint8_t, std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>>> interfaces_;
};