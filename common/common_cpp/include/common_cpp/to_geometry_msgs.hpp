#ifndef TO_GEOMETRY_MSGS_HPP
#define TO_GEOMETRY_MSGS_HPP

// ROS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS msg
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace aiformula {

geometry_msgs::msg::Point toPointMsg(const double& x, const double& y, const double& z);
geometry_msgs::msg::Quaternion toQuaternionMsg(const double& roll, const double& pitch, const double& yaw);
geometry_msgs::msg::Pose toPoseMsg(const tf2::Vector3& pos, const tf2::Vector3& rot);
geometry_msgs::msg::Pose toPoseMsg(const tf2::Vector3& pos);
geometry_msgs::msg::Vector3 toVector3Msg(const double& x, const double& y, const double& z);
geometry_msgs::msg::Vector3 toVector3Msg(const geometry_msgs::msg::Point& point);

}  // namespace aiformula

#endif  // TO_GEOMETRY_MSGS_HPP
