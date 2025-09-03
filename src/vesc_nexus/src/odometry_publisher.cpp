// src/odometry_publisher.cpp
#include "vesc_nexus/odometry_publisher.hpp"
#include <cmath>

OdometryPublisher::OdometryPublisher(
    const std::vector<VescHandler*>& vesc_handlers,
    PublishOdomFunc publish_odom,
    SendTfFunc send_tf,
    NowFunc now,
    double wheel_base, double wheel_radius)
    : vesc_handlers_(vesc_handlers),
      publish_odom_func_(std::move(publish_odom)),
      send_tf_func_(std::move(send_tf)),
      now_func_(std::move(now)),
      wheel_base_(wheel_base),
      wheel_radius_(wheel_radius),
      last_publish_time_(now())
{
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_link";
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_link";
}

void OdometryPublisher::publish() {
    rclcpp::Time now = now_func_();
    double dt = (now - last_publish_time_).seconds();
    if (dt < 0.001) return;

    double left_rpm = 0.0, right_rpm = 0.0;
    int left_count = 0, right_count = 0;

    for (auto* handler : vesc_handlers_) {
        auto state = handler->getLastState();
        std::string label = handler->getLabel();
        if (label.find("left") != std::string::npos) {
            left_rpm += state.speed_rpm;
            ++left_count;
        }
        if (label.find("right") != std::string::npos) {
            right_rpm += state.speed_rpm;
            ++right_count;
        }
    }

    if (left_count == 0 || right_count == 0) {
        RCLCPP_WARN_ONCE(rclcpp::get_logger("OdometryPublisher"), "Not enough wheels for odometry");
        return;
    }

    // Усреднение
    left_rpm /= left_count;
    right_rpm /= right_count;

    // RPM → rad/s
    double left_w = (left_rpm * 2.0 * M_PI) / 60.0;
    double right_w = (right_rpm * 2.0 * M_PI) / 60.0;

    // Линейная и угловая скорость
    double vx = (right_w + left_w) * wheel_radius_ / 2.0;
    double vtheta = (right_w - left_w) * wheel_radius_ / wheel_base_;

    // Интегрируем
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
    publish_odom_func_(odom_msg_);

    // TF
    transform_stamped_.header.stamp = now;
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();

    send_tf_func_(transform_stamped_);

    last_publish_time_ = now;
}