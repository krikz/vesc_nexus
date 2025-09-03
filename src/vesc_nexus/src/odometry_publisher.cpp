// odometry_publisher.cpp
#include "vesc_nexus/odometry_publisher.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

OdometryPublisher::OdometryPublisher(rclcpp::Node::SharedPtr node,
                                   const std::vector<VescHandler*>& vesc_handlers,
                                   double wheel_base, double wheel_radius)
    : node_(node),
      vesc_handlers_(vesc_handlers),
      wheel_base_(wheel_base),
      wheel_radius_(wheel_radius),
      last_publish_time_(node->now())
{
    odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_link";
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_link";
}

void OdometryPublisher::publish() {
    rclcpp::Time now = node_->now();
    double dt = (now - last_publish_time_).seconds();
    if (dt < 0.001) return;  // защита от слишком частых вызовов

    // Пример: предполагаем front_left и front_right как основные
    double left_rpm = 0.0, right_rpm = 0.0;
    bool found_left = false, found_right = false;

    for (auto* handler : vesc_handlers_) {
        auto state = handler->getState();
        std::string label = handler->getLabel();
        if (label.find("left") != std::string::npos) {
            left_rpm += state.speed_rpm;
            found_left = true;
        }
        if (label.find("right") != std::string::npos) {
            right_rpm += state.speed_rpm;
            found_right = true;
        }
    }

    if (!found_left || !found_right) {
        RCLCPP_WARN_ONCE(node_->get_logger(), "Not enough wheels detected for odometry");
        return;
    }

    // Среднее, если несколько колёс с одной стороны
    left_rpm /= 2.0;
    right_rpm /= 2.0;

    // RPM → угловая скорость колеса (rad/s)
    double left_w = (left_rpm * 2.0 * M_PI) / 60.0;
    double right_w = (right_rpm * 2.0 * M_PI) / 60.0;

    // Линейная и угловая скорость
    double vx = (right_w + left_w) * wheel_radius_ / 2.0;
    double vtheta = (right_w - left_w) * wheel_radius_ / wheel_base_;

    // Интегрируем положение
    double delta_x = vx * cos(theta_) * dt;
    double delta_y = vx * sin(theta_) * dt;
    double delta_theta = vtheta * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // Кватернион
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    // Заполняем одометрию
    odom_msg_.header.stamp = now;
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();

    odom_msg_.twist.twist.linear.x = vx;
    odom_msg_.twist.twist.angular.z = vtheta;

    // Публикуем
    odom_pub_->publish(odom_msg_);

    // TF
    transform_stamped_.header.stamp = now;
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform_stamped_);

    last_publish_time_ = now;
}

void OdometryPublisher::setLinearAngularVelocity(double linear, double angular) {
    vx_ = linear;
    vtheta_ = angular;
}