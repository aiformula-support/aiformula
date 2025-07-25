#ifndef OBJECT_PUBLISHER_HPP
#define OBJECT_PUBLISHER_HPP

// ROS
#include <tf2/LinearMath/Transform.h>

#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <aiformula_msgs/msg/object_info_multi_array.hpp>
#include <aiformula_msgs/msg/rect_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// Original
#include "object_publisher/tracked_object.hpp"

namespace aiformula {

class ObjectPublisher : public rclcpp::Node {
public:
    explicit ObjectPublisher();
    ~ObjectPublisher() = default;

private:
    struct InitParams {
        std::string camera_name;
        std::string camera_frame_id;
        cv::Mat camera_matrix;
        InitParams() : camera_name(), camera_frame_id(), camera_matrix(cv::Mat()) {}
    };

    void getRosParams(InitParams& init_params);
    void initValues(InitParams& init_params);
    void printParam(const InitParams& init_params) const;
    void bboxCallback(const aiformula_msgs::msg::RectMultiArray::ConstSharedPtr msg);
    bool toPositionInVehicle(const aiformula_msgs::msg::Rect& rect, tf2::Vector3& bottom_left_point,
                             tf2::Vector3& bottom_right_point) const;
    void updateOrAddObject(const tf2::Vector3& bottom_left, const tf2::Vector3& bottom_right,
                           const double& current_time);
    TrackedObject* findClosestObject(const double& obj_x, const double& obj_y);
    void deleteExpiredObjects(const double& current_time);
    void publishObjectInfo(const std_msgs::msg::Header& header, const tf2::Transform& vehicle_T_odom);

    std::string vehicle_frame_id_;
    std::string odom_frame_id_;
    bool debug_;
    double object_separation_distance_;

    cv::Mat invert_camera_matrix_;
    tf2::Transform vehicle_T_camera_;

    rclcpp::Subscription<aiformula_msgs::msg::RectMultiArray>::SharedPtr bbox_sub_;
    rclcpp::Publisher<aiformula_msgs::msg::ObjectInfoMultiArray>::SharedPtr object_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr unfilered_object_pub_;

    std::vector<TrackedObject> tracked_objects_;
};

}  // namespace aiformula

#endif  // OBJECT_PUBLISHER_HPP
