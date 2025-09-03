// vesc_can_driver_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "vesc_nexus/can_interface.hpp"
#include "vesc_nexus/vesc_handler.hpp"
#include "vesc_nexus/odometry_publisher.hpp"
#include "vesc_nexus/message_translator.hpp"
#include <vesc_msgs/msg/vesc_state.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <vector>

using vesc_msgs::msg::VescState;
using vesc_msgs::msg::VescStateStamped;
class VescCanDriverNode : public rclcpp::Node {
public:
    VescCanDriverNode(const rclcpp::NodeOptions & options)
            : rclcpp::Node("vesc_can_driver", options) {
        RCLCPP_INFO(this->get_logger(), "Initializing VESC Nexus CAN Driver Node...");

        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("vesc_ids", std::vector<int64_t>{1, 2, 3, 4});
        this->declare_parameter("wheel_labels", std::vector<std::string>{"front_left", "front_right", "rear_left", "rear_right"});
        this->declare_parameter("publish_rate", 100.0);

        std::string can_if;
        this->get_parameter("can_interface", can_if);
        std::vector<int64_t> vesc_ids;
        this->get_parameter("vesc_ids", vesc_ids);
        std::vector<std::string> labels;
        this->get_parameter("wheel_labels", labels);
        double publish_rate;
        this->get_parameter("publish_rate", publish_rate);

        if (vesc_ids.size() != labels.size()) {
            RCLCPP_FATAL(this->get_logger(), "Mismatch: %zu vesc_ids but %zu wheel_labels!", vesc_ids.size(), labels.size());
            rclcpp::shutdown();
            return;
        }

        // Инициализация CAN
        can_interface_ = std::make_unique<CanInterface>(can_if);
        can_interface_->setReceiveCallback([this](const auto& frame) {
            this->handleCanFrame(frame);
        });

        if (!can_interface_->open()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open CAN interface %s", can_if.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "CAN interface %s opened successfully.", can_if.c_str());

        // Параметры
        vesc_nexus::CommandLimits limits;
        // ... загрузи limits ...

        // Создание обработчиков
        for (size_t i = 0; i < vesc_ids.size(); ++i) {
            auto handler = std::make_shared<VescHandler>(
                static_cast<uint8_t>(vesc_ids[i]),
                labels[i],
                limits
            );

            // Установка отправки CAN
            handler->setSendCanFunc([this](const auto& f) {
                return can_interface_->sendFrame(f);
            });

            // Установка callback на обновление состояния
            handler->setStateUpdateCallback([this, label = labels[i]](const auto& state) {
                auto msg = VescStateStamped();
                msg.header.stamp = this->now();
                msg.header.frame_id = label;
                msg.state = state;

                auto it = state_pubs_.find(label);
                if (it != state_pubs_.end()) {
                    it->second->publish(msg);
                }
            });

            vesc_handlers_.push_back(handler);
            vesc_ptrs_.push_back(handler.get());
            RCLCPP_INFO(this->get_logger(), "VESC %d (%s) initialized", static_cast<int>(vesc_ids[i]), labels[i].c_str());
        }

        // Паблишеры для состояний
        for (const auto& label : labels) {
            state_pubs_[label] = this->create_publisher<VescStateStamped>(
                "sensors/motor_state/" + label, 10);
        }

        // Одометрия
        odom_publisher_ = std::make_unique<OdometryPublisher>(
            vesc_ptrs_,
            [this](const nav_msgs::msg::Odometry& msg) {
                if (!odom_pub_) {
                    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
                }
                odom_pub_->publish(msg);
            },
            [this](const geometry_msgs::msg::TransformStamped& tf) {
                if (!tf_broadcaster_) {
                    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
                }
                tf_broadcaster_->sendTransform(tf);
            },
            [this]() { return this->now(); },
            0.5,  // wheel_base
            0.1   // wheel_radius
        );
        RCLCPP_INFO(this->get_logger(), "Odometry publisher initialized.");

        // Таймер
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            [this]() {
                odom_publisher_->publish();
                for (auto& handler : vesc_handlers_) {
                    handler->requestState();
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

        RCLCPP_INFO(this->get_logger(), "✅ VESC Nexus Driver is READY and RUNNING.");
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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

    std::map<std::string, rclcpp::Publisher<VescStateStamped>::SharedPtr> state_pubs_;
};


#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(VescCanDriverNode)