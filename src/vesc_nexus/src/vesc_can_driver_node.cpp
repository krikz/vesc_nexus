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
        this->declare_parameter("wheel_radii", std::vector<double>{0.1, 0.1, 0.1, 0.1});
        this->declare_parameter("wheel_poles", std::vector<int64_t>{30, 30, 30, 30});
        this->declare_parameter("wheel_abs_min_erpm", std::vector<int64_t>{900, 900, 900, 900});
        this->declare_parameter("wheel_base", 0.3);
        
        // КАЛИБРОВКА: максимальные обороты в секунду при duty = 100%
        // Измеряется: подать duty=100%, замерить RPM из телеметрии VESC, разделить на 60
        // Пример: если при duty=100% колесо делает 900 RPM → wheel_max_rps = 15.0
        this->declare_parameter("wheel_max_rps", std::vector<double>{15.0, 15.0, 15.0, 15.0});

        std::string can_if;
        this->get_parameter("can_interface", can_if);
        std::vector<int64_t> vesc_ids;
        this->get_parameter("vesc_ids", vesc_ids);
        std::vector<std::string> labels;
        this->get_parameter("wheel_labels", labels);
        double publish_rate;
        this->get_parameter("publish_rate", publish_rate);

        std::vector<double> wheel_radii;
        this->get_parameter("wheel_radii", wheel_radii);
        std::vector<int64_t> wheel_poles;
        this->get_parameter("wheel_poles", wheel_poles);
        std::vector<int64_t> wheel_min_erpm;
        this->get_parameter("wheel_abs_min_erpm", wheel_min_erpm);
        std::vector<double> wheel_max_rps;
        this->get_parameter("wheel_max_rps", wheel_max_rps);

        this->get_parameter("wheel_base", wheel_base_);

        if (!(vesc_ids.size() == labels.size() &&
            labels.size() == wheel_radii.size() &&
            wheel_radii.size() == wheel_poles.size() &&
            wheel_poles.size() == wheel_min_erpm.size())) {
            RCLCPP_FATAL(this->get_logger(), "Mismatch in configuration array sizes!");
            rclcpp::shutdown();
            return;
        }
        
        // Дополняем wheel_max_rps если нужно
        constexpr double DEFAULT_MAX_RPS = 15.0;  // 15 об/сек по умолчанию (900 RPM)
        if (wheel_max_rps.size() < vesc_ids.size()) {
            double default_rps = wheel_max_rps.empty() ? DEFAULT_MAX_RPS : wheel_max_rps.back();
            while (wheel_max_rps.size() < vesc_ids.size()) {
                wheel_max_rps.push_back(default_rps);
            }
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

        // Создание обработчиков
        for (size_t i = 0; i < vesc_ids.size(); ++i) {
            auto handler = std::make_shared<VescHandler>(
                static_cast<uint8_t>(vesc_ids[i]),
                labels[i],
                wheel_radii[i],
                static_cast<int>(wheel_poles[i]),
                wheel_min_erpm[i],
                limits
            );

            // Установка калибровки: max_rps при duty=100%
            // Используйте tools/calibrate_max_rpm.py для автоматической калибровки
            handler->setMaxRps(wheel_max_rps[i]);

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
            wheel_base_,  // wheel_base
            0.1   // wheel_radius
        );
        RCLCPP_INFO(this->get_logger(), "Odometry publisher initialized.");

        // Таймер
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            [this]() {
                // 1. Публикуем одометрию
                odom_publisher_->publish();
                // 3. Отправляем последнюю команду (если актуальна)
                if (last_command_.valid) {
                    auto now = this->now();
                    auto dt = (now - last_command_.stamp).seconds();

                    // Если команда старше 0.5 сек — обнуляем (остановка)
                    if (dt > 0.1) {
                        last_command_.valid = false;
                        sendSpeedToWheels(0.0, 0.0);  // стоп
                    } else {
                        sendSpeedToWheels(last_command_.left_mps, last_command_.right_mps);
                    }
                } else {
                    //sendSpeedToWheels(0.0, 0.0);  // безопасная остановка
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
    // В private: секции узла
    struct WheelSpeedCommand {
        double left_mps = 0.0;
        double right_mps = 0.0;
        rclcpp::Time stamp;
        bool valid = false;
    } last_command_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double wheel_base_;
    void handleCanFrame(const can_frame& frame) {
        uint8_t sender_id = frame.can_id & 0xFF;
        for (auto& handler : vesc_handlers_) {
            if (handler->getCanId() == sender_id) {
                handler->processCanFrame(frame);
                break;
            }
        }
    }
    void sendSpeedToWheels(double left_mps, double right_mps) {
        for (auto& handler : vesc_handlers_) {
            std::string label = handler->getLabel();
            if (label.find("left") != std::string::npos) {
                handler->sendSpeed(left_mps);
            } else if (label.find("right") != std::string::npos) {
                handler->sendSpeed(right_mps);
            }
        }
    }

    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.2f|y=%.2f|z=%.2f, angular.x=%.2f|y=%.2f|z=%.2f",
                    msg->linear.x,msg->linear.y,msg->linear.z,msg->angular.x,msg->angular.y, msg->angular.z);

        double linear = msg->linear.x;
        double angular = msg->angular.z;

        double left_mps = linear - angular * wheel_base_ / 2.0;
        double right_mps = linear + angular * wheel_base_ / 2.0;

        RCLCPP_INFO(this->get_logger(), "Left: %.2f MPS, Right: %.2f MPS", left_mps, right_mps);

        // Сохраняем команду
        last_command_.left_mps = left_mps;
        last_command_.right_mps = right_mps;
        last_command_.stamp = this->now();
        last_command_.valid = true;
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