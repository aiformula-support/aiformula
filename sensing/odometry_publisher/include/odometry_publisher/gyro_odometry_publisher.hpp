#ifndef GYRO_ODOMETRY_PUBLISHER_HPP
#define GYRO_ODOMETRY_PUBLISHER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <can_msgs/msg/frame.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Original
#include "common_cpp/get_ros_parameter.hpp"
#include "common_cpp/util.hpp"
#include "odometry_publisher/linear_interporator.hpp"
#include "odometry_publisher/odometry_publisher.hpp"
#include "odometry_publisher/wheel.hpp"

namespace aiformula {

class GyroOdometryPublisher : public OdometryPublisher {
public:
    explicit GyroOdometryPublisher();
    ~GyroOdometryPublisher() = default;

private:
    void getRosParams();
    void initValues();
    void printParam() const;
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg);
    void publishOdometryCallback();

    int publish_timer_loop_duration_;
    double wheel_diameter_;
    double wheel_tread_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_msgs_;
    std::vector<can_msgs::msg::Frame::SharedPtr> can_msgs_;
};

}  // namespace aiformula

#endif  // GYRO_ODOMETRY_PUBLISHER_HPP
