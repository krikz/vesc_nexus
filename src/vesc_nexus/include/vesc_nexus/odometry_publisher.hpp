// include/vesc_nexus/odometry_publisher.hpp
#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include "vesc_handler.hpp"

class OdometryPublisher {
public:
    using PublishOdomFunc = std::function<void(const nav_msgs::msg::Odometry&)>;
    using SendTfFunc = std::function<void(const geometry_msgs::msg::TransformStamped&)>;
    using NowFunc = std::function<rclcpp::Time()>;

    OdometryPublisher(const std::vector<VescHandler*>& vesc_handlers,
                      PublishOdomFunc publish_odom,
                      SendTfFunc send_tf,
                      NowFunc now,
                      double wheel_base = 0.5,
                      double wheel_radius = 0.1);

    void publish();

private:
    std::vector<VescHandler*> vesc_handlers_;
    PublishOdomFunc publish_odom_func_;
    SendTfFunc send_tf_func_;
    NowFunc now_func_;

    double wheel_base_;
    double wheel_radius_;

    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    rclcpp::Time last_publish_time_;
    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
};