#ifndef IMAGE_COMPRESSOR_HPP
#define IMAGE_COMPRESSOR_HPP

// ROS
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// Original
#include "common_cpp/get_ros_parameter.hpp"

namespace aiformula {

class ImageCompressor : public rclcpp::Node {
public:
    explicit ImageCompressor();
    ~ImageCompressor() = default;

private:
    void getRosParams();
    void initValues();
    void printParam() const;
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const;

    int jpeg_quality_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
};

}  // namespace aiformula

#endif  // IMAGE_COMPRESSOR_HPP
