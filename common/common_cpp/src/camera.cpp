#include "common_cpp/camera.hpp"

namespace aiformula {

/**
 * @brief Get Camera Parameter from rosparam.
 *
 * @param[in] node_ptr node pointer
 * @param[in] camera_name Camera Name
 * @param[out] camera_matrix Camera Matrix
 * @param[out] image_size Image Size
 *
 * @note Usage: `getCameraParams(this, "zedx", camera_matrix, &image_size);`
 */
void getCameraParams(rclcpp::Node* const node_ptr, const std::string& camera_name, cv::Mat& camera_matrix,
                     cv::Size* image_size) {
    camera_matrix = cv::Mat::eye(3, 3, CV_32F);
    camera_matrix.at<float>(0, 0) = getRosParameter<double>(node_ptr, camera_name + ".focal_length.x");
    camera_matrix.at<float>(1, 1) = getRosParameter<double>(node_ptr, camera_name + ".focal_length.y");
    camera_matrix.at<float>(0, 2) = getRosParameter<double>(node_ptr, camera_name + ".center_point.x");
    camera_matrix.at<float>(1, 2) = getRosParameter<double>(node_ptr, camera_name + ".center_point.y");

    if (image_size) {
        image_size->width = getRosParameter<int>(node_ptr, camera_name + ".size.width");
        image_size->height = getRosParameter<int>(node_ptr, camera_name + ".size.height");
    }
}  // getCameraParams

}  // namespace aiformula
