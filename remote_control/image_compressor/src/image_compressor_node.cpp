#include "image_compressor/image_compressor.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aiformula::ImageCompressor>());
    rclcpp::shutdown();
    return 0;
}
