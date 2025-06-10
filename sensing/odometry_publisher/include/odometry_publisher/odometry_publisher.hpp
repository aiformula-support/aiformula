#ifndef ODOMETRY_PUBLISHER_HPP
#define ODOMETRY_PUBLISHER_HPP

// ROS
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Original
#include "common_cpp/get_ros_parameter.hpp"
#include "common_cpp/to_geometry_msgs.hpp"
#include "common_cpp/util.hpp"

namespace aiformula {

class OdometryPublisher : public rclcpp::Node {
public:
    explicit OdometryPublisher(const std::string& node_name);
    ~OdometryPublisher() = default;

protected:
    double calcTimeDelta(const builtin_interfaces::msg::Time& msg_time);
    void updatePosition(const double& vehicle_linear_velocity, const double& dt);
    nav_msgs::msg::Odometry createOdometryMsg(const builtin_interfaces::msg::Time& msg_time) const;
    void broadcastTf(const nav_msgs::msg::Odometry& odom);

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    geometry_msgs::msg::Point pos_;
    geometry_msgs::msg::Vector3 vehicle_linear_velocity_;
    double yaw_angle_;
    double yaw_rate_;

private:
    void getRosParams();
    void initValues();
    void printParam() const;

    std::string odom_frame_id_;
    std::string vehicle_frame_id_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> odometry_br_;
    double prev_time_;
};

}  // namespace aiformula

#endif  // ODOMETRY_PUBLISHER_HPP
