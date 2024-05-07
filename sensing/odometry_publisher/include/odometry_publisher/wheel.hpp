#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <can_msgs/msg/frame.hpp>

namespace aiformula {
namespace odometry_publisher {

template <typename T>
struct WheelRates {
    explicit WheelRates(const T& left, const T& right) : left(left), right(right) {}
    WheelRates<double> operator*(const double& rhs) const { return WheelRates<double>(left * rhs, right * rhs); }
    T left, right;
};
const double SECOND_TO_MINUTE = 0.016667;  // = 1/60
const double DEGREE_TO_RADIAN = M_PI / 180.0;
const double RADIAN_TO_DEGREE = 180.0 / M_PI;

const int RPM_ID = 1809;

inline WheelRates<double> calcEachWheelVelocity(const can_msgs::msg::Frame::SharedPtr& can_msg,
                                                const double& wheel_diameter) {
    const WheelRates<unsigned int> rpm(can_msg->data[4], can_msg->data[0]);
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
