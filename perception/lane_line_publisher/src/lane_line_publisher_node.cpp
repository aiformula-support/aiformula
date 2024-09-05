#include "lane_line_publisher/lane_line_publisher.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aiformula::LaneLinePublisher>());
    rclcpp::shutdown();
    return 0;
}
