#include "odometry_publisher/wheel_odometry_publisher.hpp"

namespace aiformula {

WheelOdometryPublisher::WheelOdometryPublisher() : OdometryPublisher("wheel_odometry_publisher") {
    getRosParams();
    initValues();
    printParam();
}

void WheelOdometryPublisher::getRosParams() {
    // From 'wheel.yaml'
    wheel_diameter_ = getRosParameter<double>(this, "wheel.diameter");
    wheel_tread_ = getRosParameter<double>(this, "wheel.tread");
}

void WheelOdometryPublisher::initValues() {
    const int buffer_size = 10;
    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "sub_can", buffer_size, std::bind(&WheelOdometryPublisher::canFrameCallback, this, std::placeholders::_1));
}

void WheelOdometryPublisher::printParam() const {
    RCLCPP_INFO(this->get_logger(), "[%s] ===============", __func__);
    RCLCPP_INFO(this->get_logger(), "(wheel.yaml)");
    RCLCPP_INFO(this->get_logger(), "  wheel_diameter_ = %.3lf [m]", wheel_diameter_);
    RCLCPP_INFO(this->get_logger(), "  wheel_tread_    = %.3lf [m]", wheel_tread_);
    RCLCPP_INFO(this->get_logger(), "============================\n");
}

void WheelOdometryPublisher::canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Can Frame !");
    if (msg->id != odometry_publisher::RPM_ID) return;

    const double vehicle_linear_velocity = odometry_publisher::calcLinearVelocity(msg, wheel_diameter_);
    yaw_rate_ = odometry_publisher::calcYawRateFromWheelRpm(msg, wheel_diameter_, wheel_tread_);
    const double dt = calcTimeDelta(msg->header.stamp);
    yaw_angle_ += yaw_rate_ * dt;
    updatePosition(vehicle_linear_velocity, dt);
    nav_msgs::msg::Odometry odometry = createOdometryMsg(msg->header.stamp);
    odometry_pub_->publish(odometry);
    broadcastTf(odometry);
    RCLCPP_INFO_ONCE(this->get_logger(), "Publish Odometry !");
}

}  // namespace aiformula
