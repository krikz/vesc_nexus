// odometry_publisher.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "vesc_handler.hpp"
#include <vector>

class OdometryPublisher {
public:
    OdometryPublisher(rclcpp::Node::SharedPtr node,
                      const std::vector<VescHandler*>& vesc_handlers,
                      double wheel_base = 0.5,
                      double wheel_radius = 0.1);

    void publish();

    void setLinearAngularVelocity(double linear, double angular);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::vector<VescHandler*> vesc_handlers_;
    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::TransformStamped transform_stamped_;

    double wheel_base_;     // расстояние между колёсами (для diff drive)
    double wheel_radius_;   // радиус колеса в метрах

    rclcpp::Time last_publish_time_;
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    double vx_ = 0.0, vy_ = 0.0, vtheta_ = 0.0;
};