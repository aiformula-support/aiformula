#include "odometry_publisher/gnss_odometry_publisher.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aiformula::GnssOdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
