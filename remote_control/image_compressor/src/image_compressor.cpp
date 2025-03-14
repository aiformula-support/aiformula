#include "image_compressor/image_compressor.hpp"

namespace aiformula {

ImageCompressor::ImageCompressor() : Node("image_compressor"), jpeg_quality_(10) {
    getRosParams();
    initValues();
    printParam();
}

void ImageCompressor::getRosParams() {
    // image_compressor.yaml
    jpeg_quality_ = getRosParameter<int>(this, "jpeg_quality");
}

void ImageCompressor::initValues() {
    // Subscriber & Publisher
    const int buffer_size = 10;
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "sub_image", buffer_size, std::bind(&ImageCompressor::imageCallback, this, std::placeholders::_1));
    compressed_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("pub_image", buffer_size);
}

void ImageCompressor::printParam() const {
    RCLCPP_INFO(this->get_logger(), "[%s] ===============", __func__);
    RCLCPP_INFO(this->get_logger(), "(image_compressor.yaml)");
    RCLCPP_INFO(this->get_logger(), "  jpeg_quality_ : %d", jpeg_quality_);
    RCLCPP_INFO(this->get_logger(), "============================\n");
}

void ImageCompressor::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Image !");
    cv::Mat raw_image;
    try {
        raw_image = cv_bridge::toCvShare(msg)->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[%s] cv_bridge exception: %s", __func__, e.what());
        return;
    }
    sensor_msgs::msg::CompressedImage::UniquePtr pub_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    pub_msg->header = msg->header;
    pub_msg->format = "jpeg";
    const std::vector<int> compression_params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".jpg", raw_image, pub_msg->data, compression_params)) {
        RCLCPP_ERROR(this->get_logger(), "[%s] Encoding failed.", __func__);
        return;
    }
    compressed_image_pub_->publish(std::move(pub_msg));
    RCLCPP_INFO_ONCE(this->get_logger(), "Publish Compressed Image !");
}

}  // namespace aiformula
