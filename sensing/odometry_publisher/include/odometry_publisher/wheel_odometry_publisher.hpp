#ifndef WHEEL_ODOMETRY_PUBLISHER_HPP
#define WHEEL_ODOMETRY_PUBLISHER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <can_msgs/msg/frame.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Original
#include "common_cpp/get_ros_parameter.hpp"
#include "odometry_publisher/odometry_publisher.hpp"
#include "odometry_publisher/wheel.hpp"

namespace aiformula {

class WheelOdometryPublisher : public OdometryPublisher {
public:
    explicit WheelOdometryPublisher();
    ~WheelOdometryPublisher() = default;

private:
    void getRosParams();
    void initValues();
    void printParam() const;
    void canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg);

    double wheel_diameter_;
    double wheel_tread_;

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
};

}  // namespace aiformula

#endif  // WHEEL_ODOMETRY_PUBLISHER_HPP
