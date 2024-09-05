#ifndef LANE_LINE_PUBLISHER_HPP
#define LANE_LINE_PUBLISHER_HPP

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common_cpp/camera.hpp"
#include "common_cpp/get_ros_parameter.hpp"
#include "lane_line_publisher/lane_line.hpp"
#include "lane_line_publisher/lane_pixel_finder.hpp"

namespace aiformula {

class LaneLinePublisher : public rclcpp::Node {
public:
    LaneLinePublisher();
    ~LaneLinePublisher() = default;

private:
    void initMembers();
    void initConnections();

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const;
    void findLaneLines(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp,
                       LaneLines& lane_lines) const;
    void publishAnnotatedMask(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp,
                              const LaneLines& lane_lines) const;
    void publishContourPoints(const std::vector<std::vector<Eigen::Vector3d>>& contour_points,
                              const builtin_interfaces::msg::Time& timestamp) const;
    void publishLaneLines(const LaneLines& lane_lines, const builtin_interfaces::msg::Time& timestamp) const;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_mask_image_pub_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> lane_line_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> contour_point_pubs_;

    LanePixelFinder::ConstPtr lane_pixel_finder_;
    CubicLineFitter::ConstPtr cubic_line_fitter_;

    bool debug_;
    std::string vehicle_frame_id_;
    double xmin_, xmax_, ymin_, ymax_, spacing_;  // [m]
    cv::Mat camera_matrix_;
    tf2::Transform vehicle_T_camera_;
};

}  // namespace aiformula

#endif  // LANE_LINE_PUBLISHER_HPP
