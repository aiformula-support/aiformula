#ifndef TF2_TRANSFORM_HPP
#define TF2_TRANSFORM_HPP

// ROS
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

namespace aiformula {

tf2::Transform getTf2Transform(rclcpp::Node* const node_ptr, const std::string& dst_frame_id,
                               const std::string& src_frame_id);
bool pixelToPoint(const cv::Point2f& pixel, const cv::Mat& invert_camera_matrix, const tf2::Transform& vehicle_T_camera,
                  tf2::Vector3& vehicle_point);
std::vector<tf2::Vector3> pixelsToPoints(const std::vector<cv::Point2f>& pixels, const cv::Mat& invert_camera_matrix,
                                         const tf2::Transform& vehicle_T_camera);
bool pointToPixel(const tf2::Vector3& vehicle_point, const cv::Mat& camera_matrix,
                  const tf2::Transform& camera_T_vehicle, cv::Point2f& pixel, const cv::Rect* valid_roi = nullptr);
std::vector<cv::Point2f> pointsToPixels(const std::vector<tf2::Vector3>& vehicle_points, const cv::Mat& camera_matrix,
                                        const tf2::Transform& camera_T_vehicle, const cv::Rect* valid_roi = nullptr);
}  // namespace aiformula

#endif  // TF2_TRANSFORM_HPP
