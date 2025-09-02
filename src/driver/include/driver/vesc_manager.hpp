#ifndef VESC_MANAGER_HPP_
#define VESC_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class VescManager {
public:
    // Конструктор: принимает указатель на ноду
    explicit VescManager(rclcpp::Node* node);

    // Метод для добавления VESC
    void addVesc(const std::string& interface_name, uint8_t id, const std::string& label);

private:
    rclcpp::Node* node_; // Указатель на ноду

    // Структура для хранения информации о VESC
    struct VescInfo {
        uint8_t id;
        std::string label;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber;
    };

    // Карта для хранения интерфейсов и их VESC
    std::map<std::string, std::vector<VescInfo>> interfaces_;
};

#endif // VESC_MANAGER_HPP_