#ifndef LINEAR_INTERPOLATOR_HPP
#define LINEAR_INTERPOLATOR_HPP

// ROS
#include <tf2/LinearMath/Quaternion.h>

namespace aiformula {
namespace odometry_publisher {

class LinearInterpolator {
public:
    explicit LinearInterpolator(const double& left_time, const double& right_time, const double& left_value,
                                const double& right_value)
        : left_time_(left_time), right_time_(right_time), left_value_(left_value), right_value_(right_value) {
        slope_ = (right_value_ - left_value_) / (right_time_ - left_time_);
    }
    ~LinearInterpolator() = default;
    double valueAt(const double& time) const { return left_value_ + slope_ * (time - left_time_); }

private:
    double left_time_, right_time_;
    double left_value_, right_value_;
    double slope_;
};

class SphericalLinearInterpolator {
public:
    explicit SphericalLinearInterpolator(const double& left_time, const double& right_time,
                                         const tf2::Quaternion& left_value, const tf2::Quaternion& right_value)
        : left_time_(left_time), right_time_(right_time), left_value_(left_value), right_value_(right_value) {}
    ~SphericalLinearInterpolator() = default;
    tf2::Quaternion valueAt(const double& time) const {
        const double ratio = (time - left_time_) / (right_time_ - left_time_);
        return left_value_.slerp(right_value_, ratio);
    }

private:
    double left_time_, right_time_;
    tf2::Quaternion left_value_, right_value_;
};

}  // namespace odometry_publisher
}  // namespace aiformula

#endif  // LINEAR_INTERPOLATOR_HPP
