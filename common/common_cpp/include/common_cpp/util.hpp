#ifndef UTIL_HPP
#define UTIL_HPP

// ROS
#include <tf2/impl/utils.h>

// ROS msg
#include <geometry_msgs/msg/quaternion.hpp>

namespace aiformula {

inline double toTimeStampDouble(const builtin_interfaces::msg::Time& msg_stamp) {
    return msg_stamp.sec + static_cast<double>(msg_stamp.nanosec) / 1e9;
}

inline double getYaw(const geometry_msgs::msg::Quaternion& quat_msg) {
    return tf2::impl::getYaw(tf2::impl::toQuaternion(quat_msg));
}

}  // namespace aiformula

#endif  // UTIL_HPP
