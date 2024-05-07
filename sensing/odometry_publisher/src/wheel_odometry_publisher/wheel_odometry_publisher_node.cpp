#include "odometry_publisher/wheel_odometry_publisher.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aiformula::WheelOdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
