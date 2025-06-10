#ifndef WHEEL_HPP
#define WHEEL_HPP

// ROS msg
#include <can_msgs/msg/frame.hpp>

// C, C++
#include <cstring>

namespace aiformula {
namespace odometry_publisher {

union Uint8ArrayToInt {
    uint8_t arr[4];
    int val;
    Uint8ArrayToInt(uint8_t arr_in[4]) { std::memcpy(arr, arr_in, sizeof(arr)); }
};

template <typename T>
struct DriveWheel {
    explicit DriveWheel(const T& left_in, const T& right_in) : left(left_in), right(right_in) {}
    DriveWheel<double> operator*(const double& rhs) const { return DriveWheel<double>(left * rhs, right * rhs); }
    T left, right;
};

const double SECOND_TO_MINUTE = 0.016667;  // = 1/60
const double DEGREE_TO_RADIAN = M_PI / 180.0;
const double RADIAN_TO_DEGREE = 180.0 / M_PI;

const int RPM_ID = 1809;

inline DriveWheel<double> calcEachWheelVelocity(const can_msgs::msg::Frame::SharedPtr& can_msg,
                                                const double& wheel_diameter) {
    // Right: can_msg->data[0]-[3],  Left: can_msg->data[4]-[7]
    const DriveWheel<Uint8ArrayToInt> rpm_union(&(can_msg->data[4]), &(can_msg->data[0]));
    const DriveWheel<int> rpm(rpm_union.left.val, rpm_union.right.val);
    const auto wheel_circumference = wheel_diameter * M_PI;
    return rpm * SECOND_TO_MINUTE * wheel_circumference;
}

inline double calcLinearVelocity(const can_msgs::msg::Frame::SharedPtr& can_msg, const double& wheel_diameter) {
    const auto vel = calcEachWheelVelocity(can_msg, wheel_diameter);
    return (vel.left + vel.right) * 0.5;
}

inline double calcYawRateFromWheelRpm(const can_msgs::msg::Frame::SharedPtr& can_msg, const double& wheel_diameter,
                                      const double& wheel_tread) {
    const auto vel = calcEachWheelVelocity(can_msg, wheel_diameter);
    return (vel.right - vel.left) / wheel_tread;
}

}  // namespace odometry_publisher
}  // namespace aiformula

#endif  // WHEEL_HPP
