#include "object_publisher/object_publisher.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aiformula::ObjectPublisher>());
    rclcpp::shutdown();
    return 0;
}
