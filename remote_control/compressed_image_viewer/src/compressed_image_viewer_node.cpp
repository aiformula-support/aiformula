#include "compressed_image_viewer/compressed_image_viewer.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    const auto compressed_image_viewer = std::make_shared<aiformula::CompressedImageViewer>();
    compressed_image_viewer->setup();
    rclcpp::spin(compressed_image_viewer);
    rclcpp::shutdown();
    return 0;
}
