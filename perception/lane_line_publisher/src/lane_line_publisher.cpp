#include "lane_line_publisher/lane_line_publisher.hpp"

namespace aiformula {

LaneLinePublisher::LaneLinePublisher() : Node("lane_line_publisher") {
    initMembers();
    initConnections();
    RCLCPP_INFO(get_logger(), "Launched %s", get_name());
}

void LaneLinePublisher::initMembers() {
    debug_ = getRosParameter<bool>(this, "debug");
    vehicle_frame_id_ = getRosParameter<std::string>(this, "vehicle_frame_id");
    xmin_ = getRosParameter<double>(this, "lane_line_publisher.roi.xmin");
    xmax_ = getRosParameter<double>(this, "lane_line_publisher.roi.xmax");
    ymin_ = getRosParameter<double>(this, "lane_line_publisher.roi.ymin");
    ymax_ = getRosParameter<double>(this, "lane_line_publisher.roi.ymax");
    spacing_ = getRosParameter<double>(this, "lane_line_publisher.spacing");
    const auto camera_frame_id = getRosParameter<std::string>(this, "camera_frame_id");
    const auto camera_name = getRosParameter<std::string>(this, "camera_name");
    const auto min_area = getRosParameter<int>(this, "lane_pixel_finder.min_area");
    const auto tolerance = getRosParameter<int>(this, "lane_pixel_finder.tolerance");

    getCameraParams(this, camera_name, camera_matrix_);
    camera_matrix_ = camera_matrix_.inv();
    vehicle_T_camera_ = getTf2Transform(this, vehicle_frame_id_, camera_frame_id);

    lane_pixel_finder_ = std::make_shared<const LanePixelFinder>(min_area, tolerance);
    cubic_line_fitter_ = std::make_shared<const CubicLineFitter>(xmin_, xmax_, spacing_);
}

void LaneLinePublisher::initConnections() {
    const auto queue_size = getRosParameter<int>(this, "lane_line_publisher.queue_size");
    mask_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "mask_image", queue_size, std::bind(&LaneLinePublisher::imageCallback, this, std::placeholders::_1));
    lane_line_pubs_ = {create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/left", queue_size),
                       create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/right", queue_size),
                       create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/center", queue_size)};

    if (!debug_) return;

    annotated_mask_image_pub_ = create_publisher<sensor_msgs::msg::Image>("annotated_mask_image", queue_size);
    contour_point_pubs_ = {create_publisher<sensor_msgs::msg::PointCloud2>("contour_points/left", queue_size),
                           create_publisher<sensor_msgs::msg::PointCloud2>("contour_points/right", queue_size),
                           create_publisher<sensor_msgs::msg::PointCloud2>("contour_points/center", queue_size)};
}

void LaneLinePublisher::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    RCLCPP_DEBUG(get_logger(), "Recieved image [%ix%i]", msg->width, msg->height);

    auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
    if (cv_img->image.empty()) {
        RCLCPP_WARN(get_logger(), "Recieved empty image");
        return;
    }

    LaneLines lane_lines;
    findLaneLines(cv_img->image, msg->header.stamp, lane_lines);
    publishLaneLines(lane_lines, msg->header.stamp);
}

void LaneLinePublisher::findLaneLines(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp,
                                      LaneLines& lane_lines) const {
    lane_pixel_finder_->findLanePixels(mask, lane_lines);
    if (debug_) publishAnnotatedMask(mask, timestamp, lane_lines);

    std::vector<std::vector<Eigen::Vector3d>> contour_points;
    std::vector<LaneLine*> lane_line_ptrs = {&lane_lines.left, &lane_lines.right, &lane_lines.center};
    for (auto* lane_line : lane_line_ptrs) {
        lane_line->toVehicleFrame(camera_matrix_, vehicle_T_camera_);
        if (debug_) contour_points.emplace_back(lane_line->points);
        lane_line->cropToRoi(xmin_, xmax_, ymin_, ymax_);
        lane_line->respacePoints(spacing_);
        lane_line->fitPoints(cubic_line_fitter_);
    }

    if (debug_) publishContourPoints(contour_points, timestamp);
}

void LaneLinePublisher::publishAnnotatedMask(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp,
                                             const LaneLines& lane_lines) const {
    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = timestamp;
    cv_img.encoding = "bgr8";
    cv_img.image = lane_pixel_finder_->visualizeLanePixels(mask, lane_lines);

    sensor_msgs::msg::Image msg;
    cv_img.toImageMsg(msg);
    annotated_mask_image_pub_->publish(std::move(msg));
}

void LaneLinePublisher::publishContourPoints(const std::vector<std::vector<Eigen::Vector3d>>& contour_points,
                                             const builtin_interfaces::msg::Time& timestamp) const {
    for (int i = 0; i < NUM_LANE_LINES; ++i) {
        const auto& points = contour_points[i];
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (const auto& point : points) {
            auto& p = cloud.points.emplace_back();
            p.x = point.x();
            p.y = point.y();
        }

        sensor_msgs::msg::PointCloud2 pcl_msg;
        pcl::toROSMsg(cloud, pcl_msg);
        pcl_msg.header.stamp = timestamp;
        pcl_msg.header.frame_id = vehicle_frame_id_;
        contour_point_pubs_[i]->publish(pcl_msg);
    }
}

void LaneLinePublisher::publishLaneLines(const LaneLines& lane_lines,
                                         const builtin_interfaces::msg::Time& timestamp) const {
    const std::vector<const LaneLine*> lane_line_ptrs = {&lane_lines.left, &lane_lines.right, &lane_lines.center};

    for (int i = 0; i < NUM_LANE_LINES; ++i) {
        const auto& lane_line_points = lane_line_ptrs[i]->points;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (const auto& point : lane_line_points) {
            auto& p = cloud.points.emplace_back();
            p.x = point.x();
            p.y = point.y();
        }

        sensor_msgs::msg::PointCloud2 pcl_msg;
        pcl::toROSMsg(cloud, pcl_msg);
        pcl_msg.header.stamp = timestamp;
        pcl_msg.header.frame_id = vehicle_frame_id_;
        lane_line_pubs_[i]->publish(pcl_msg);
    }
}

}  // namespace aiformula
