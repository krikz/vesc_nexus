// vesc_can_driver_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "vesc_nexus/can_interface.hpp"
#include "vesc_nexus/vesc_handler.hpp"
#include "vesc_nexus/odometry_publisher.hpp"
#include "vesc_nexus/message_translator.hpp"
#include <memory>
#include <vector>

class VescCanDriverNode : public rclcpp::Node {
public:
    VescCanDriverNode() : Node("vesc_can_driver") {
        // Объявление параметров
        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("vesc_ids", std::vector<int>{1, 2, 3, 4});
        this->declare_parameter("wheel_labels", std::vector<std::string>{"front_left", "front_right", "rear_left", "rear_right"});
        this->declare_parameter("publish_rate", 100.0);

        // Загрузка
        std::string can_if;
        this->get_parameter("can_interface", can_if);
        std::vector<int> vesc_ids;
        this->get_parameter("vesc_ids", vesc_ids);
        std::vector<std::string> labels;
        this->get_parameter("wheel_labels", labels);
        double publish_rate;
        this->get_parameter("publish_rate", publish_rate);

        // Проверка
        if (vesc_ids.size() != labels.size()) {
            RCLCPP_FATAL(this->get_logger(), "Mismatch between vesc_ids and wheel_labels count");
            rclcpp::shutdown();
            return;
        }

        // Инициализация CAN
        can_interface_ = std::make_unique<CanInterface>(can_if);
        can_interface_->setReceiveCallback([this](const auto& frame) {
            this->handleCanFrame(frame);
        });

        if (!can_interface_->open()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open CAN interface");
            rclcpp::shutdown();
            return;
        }

        // Создание обработчиков
        vesc_nexus::CommandLimits limits;
        this->declare_parameter("duty_cycle_min", -0.5);
        this->declare_parameter("duty_cycle_max", 0.5);
        this->get_parameter("duty_cycle_min", limits.duty_cycle_min);
        this->get_parameter("duty_cycle_max", limits.duty_cycle_max);
        // ... аналогично для других параметров

        for (size_t i = 0; i < vesc_ids.size(); ++i) {
            auto handler = std::make_shared<VescHandler>(
                static_cast<uint8_t>(vesc_ids[i]),
                labels[i],
                shared_from_this(),
                limits
            );
            handler->setSendCanFunc([this](const auto& f) {
                return can_interface_->sendFrame(f);
            });
            vesc_handlers_.push_back(handler);
            vesc_ptrs_.push_back(handler.get());
        }

        // Одометрия
        odom_publisher_ = std::make_unique<OdometryPublisher>(
            shared_from_this(), vesc_ptrs_
        );

        // Таймер публикации и опроса
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            [this]() {
                odom_publisher_->publish();
                for (auto& handler : vesc_handlers_) {
                    handler->requestState();  // Опрос состояния
                }
            }
        );

        // Подписка на cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                this->onCmdVel(msg);
            }
        );
    }

private:
    void handleCanFrame(const can_frame& frame) {
        uint8_t sender_id = frame.can_id & 0xFF;
        for (auto& handler : vesc_handlers_) {
            if (handler->getCanId() == sender_id) {
                handler->processCanFrame(frame);
                break;
            }
        }
    }

    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        double wheel_base = 0.5;
        double left_speed = linear - angular * wheel_base / 2.0;
        double right_speed = linear + angular * wheel_base / 2.0;

        for (auto& handler : vesc_handlers_) {
            std::string label = handler->getLabel();
            if (label.find("left") != std::string::npos) {
                handler->sendSpeed(left_speed * 60.0 / (2.0 * M_PI * 0.1) * 7);  // RPM (пример)
            } else if (label.find("right") != std::string::npos) {
                handler->sendSpeed(right_speed * 60.0 / (2.0 * M_PI * 0.1) * 7);
            }
        }
    }

    std::unique_ptr<CanInterface> can_interface_;
    std::vector<std::shared_ptr<VescHandler>> vesc_handlers_;
    std::vector<VescHandler*> vesc_ptrs_;
    std::unique_ptr<OdometryPublisher> odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VescCanDriverNode>());
    rclcpp::shutdown();
    return 0;
}