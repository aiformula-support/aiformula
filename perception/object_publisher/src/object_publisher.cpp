#include "object_publisher/object_publisher.hpp"

#include "common_cpp/camera.hpp"
#include "common_cpp/get_ros_parameter.hpp"
#include "common_cpp/tf2_transform.hpp"
#include "common_cpp/to_geometry_msgs.hpp"
#include "common_cpp/util.hpp"

namespace aiformula {

ObjectPublisher::ObjectPublisher()
    : Node("object_publisher"),
      vehicle_frame_id_(),
      odom_frame_id_(),
      debug_(false),
      object_separation_distance_(5.0),
      invert_camera_matrix_(cv::Mat()),
      vehicle_T_camera_(tf2::Transform()),
      tracked_objects_(std::vector<TrackedObject>()) {
    InitParams init_params;
    getRosParams(init_params);
    initValues(init_params);
    printParam(init_params);
}

void ObjectPublisher::getRosParams(InitParams& init_params) {
    // From 'launch file'
    init_params.camera_name = getRosParameter<std::string>(this, "camera_name");
    init_params.camera_frame_id = getRosParameter<std::string>(this, "camera_frame_id");
    vehicle_frame_id_ = getRosParameter<std::string>(this, "vehicle_frame_id");
    odom_frame_id_ = getRosParameter<std::string>(this, "odom_frame_id");
    debug_ = getRosParameter<bool>(this, "debug");

    // From 'object_publisher.yaml'
    object_separation_distance_ = getRosParameter<double>(this, "object_separation_distance");

    TrackedObject::initStaticMembers(this);
}

void ObjectPublisher::initValues(InitParams& init_params) {
    // Subscriber & Publisher
    const int buffer_size = 10;
    bbox_sub_ = this->create_subscription<aiformula_msgs::msg::RectMultiArray>(
        "sub_bbox", buffer_size, std::bind(&ObjectPublisher::bboxCallback, this, std::placeholders::_1));
    object_pub_ = this->create_publisher<aiformula_msgs::msg::ObjectInfoMultiArray>("pub_object", buffer_size);
    if (debug_)
        unfilered_object_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseArray>("pub_unfiltered_object", buffer_size);

    getCameraParams(this, init_params.camera_name, init_params.camera_matrix);
    invert_camera_matrix_ = init_params.camera_matrix.inv();
    vehicle_T_camera_ = getTf2Transform(this, vehicle_frame_id_, init_params.camera_frame_id);
}

void ObjectPublisher::printParam(const InitParams& init_params) const {
    std::ostringstream log_stream;
    const auto formatter = createFormatter(20);
    const auto& trans = vehicle_T_camera_.getOrigin();
    const auto& rot = vehicle_T_camera_.getRotation();
    log_stream << "\n[" << __func__ << "] " << std::string(57, '=') << "\n"
               << "(launch)\n"
               << formatter("camera_name") << init_params.camera_name << "\n"
               << formatter("camera_frame_id") << init_params.camera_frame_id << "\n"
               << formatter("vehicle_frame_id_") << vehicle_frame_id_ << "\n"
               << formatter("odom_frame_id_") << odom_frame_id_ << "\n"
               << formatter("debug_") << (debug_ ? "true" : "false") << "\n"
               << "\n(object_publisher.yaml)\n"
               << std::fixed << std::setprecision(1) << formatter("object_separation_distance_")
               << object_separation_distance_ << " [m]\n"
               << "\n(initValues)\n"
               << formatter("camera_matrix") << "[" << init_params.camera_matrix.row(0) << "\n"
               << std::string(27, ' ') << init_params.camera_matrix.row(1) << "\n"
               << std::string(27, ' ') << init_params.camera_matrix.row(2) << "]\n"
               << std::setprecision(2) << formatter("trans") << "(" << trans.x() << ", " << trans.y() << ", "
               << trans.z() << ") [m]\n"
               << formatter("rot") << "(" << rot.x() << ", " << rot.y() << ", " << rot.z() << ", " << rot.w() << ")\n"
               << std::string(70, '=') << "\n";
    RCLCPP_DEBUG(this->get_logger(), "%s", log_stream.str().c_str());
    TrackedObject::printStaticMembers();
}

void ObjectPublisher::bboxCallback(const aiformula_msgs::msg::RectMultiArray::ConstSharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe BBox Rect !");
    std_msgs::msg::Header header = msg->header;
    header.frame_id = vehicle_frame_id_;

    geometry_msgs::msg::PoseArray unfiltered_object_msg;
    if (debug_) unfiltered_object_msg.header = header;

    const auto odom_T_vehicle = getTf2Transform(this, odom_frame_id_, vehicle_frame_id_);
    const auto current_time = toTimeStampDouble(msg->header.stamp);
    for (const auto& rect : msg->rects) {
        tf2::Vector3 bottom_left_in_vehicle, bottom_right_in_vehicle;
        if (!toPositionInVehicle(rect, bottom_left_in_vehicle, bottom_right_in_vehicle)) continue;
        const auto bottom_left_in_odom = odom_T_vehicle * bottom_left_in_vehicle;
        const auto bottom_right_in_odom = odom_T_vehicle * bottom_right_in_vehicle;
        updateOrAddObject(bottom_left_in_odom, bottom_right_in_odom, current_time);

        const float center_x_in_vehicle = (bottom_left_in_vehicle.x() + bottom_right_in_vehicle.x()) * 0.5;
        const float center_y_in_vehicle = (bottom_left_in_vehicle.y() + bottom_right_in_vehicle.y()) * 0.5;
        if (debug_)
            unfiltered_object_msg.poses.emplace_back(
                toPoseMsg(tf2::Vector3(center_x_in_vehicle, center_y_in_vehicle, 0.0)));
    }
    deleteExpiredObjects(current_time);
    publishObjectInfo(header, odom_T_vehicle.inverse());
    if (debug_) unfilered_object_pub_->publish(unfiltered_object_msg);
}

bool ObjectPublisher::toPositionInVehicle(const aiformula_msgs::msg::Rect& rect, tf2::Vector3& bottom_left_point,
                                          tf2::Vector3& bottom_right_point) const {
    const cv::Point2f bottom_left(rect.x, rect.y + rect.height);
    const cv::Point2f bottom_right(rect.x + rect.width, rect.y + rect.height);
    return pixelToPoint(bottom_left, invert_camera_matrix_, vehicle_T_camera_, bottom_left_point) &&
           pixelToPoint(bottom_right, invert_camera_matrix_, vehicle_T_camera_, bottom_right_point);
}

void ObjectPublisher::updateOrAddObject(const tf2::Vector3& bottom_left, const tf2::Vector3& bottom_right,
                                        const double& current_time) {
    const float center_x = (bottom_left.x() + bottom_right.x()) * 0.5;
    const float center_y = (bottom_left.y() + bottom_right.y()) * 0.5;
    TrackedObject* closest_object = findClosestObject(center_x, center_y);
    if (closest_object) {
        closest_object->update(bottom_left.x(), bottom_left.y(), bottom_right.x(), bottom_right.y(), current_time);
    } else {
        static unsigned int next_object_id = 0;
        tracked_objects_.emplace_back(next_object_id++, bottom_left.x(), bottom_left.y(), bottom_right.x(),
                                      bottom_right.y(), current_time);
    }
}

TrackedObject* ObjectPublisher::findClosestObject(const double& obj_x, const double& obj_y) {
    if (tracked_objects_.empty()) return nullptr;

    std::vector<float> distances_squared;
    for (const auto& tracked_object : tracked_objects_)
        distances_squared.emplace_back(tracked_object.computeDistanceSquared(obj_x, obj_y));

    const auto closest_distance_squared_it{std::min_element(distances_squared.begin(), distances_squared.end())};

    const double max_distance_squared{object_separation_distance_ * object_separation_distance_};
    if (*closest_distance_squared_it > max_distance_squared) return nullptr;

    const auto closest_object_index{std::distance(distances_squared.begin(), closest_distance_squared_it)};
    return &tracked_objects_.at(closest_object_index);
}

void ObjectPublisher::deleteExpiredObjects(const double& current_time) {
    tracked_objects_.erase(std::remove_if(tracked_objects_.begin(), tracked_objects_.end(),
                                          [current_time](TrackedObject& obj) { return obj.isExpired(current_time); }),
                           tracked_objects_.end());
}

void ObjectPublisher::publishObjectInfo(const std_msgs::msg::Header& header, const tf2::Transform& vehicle_T_odom) {
    aiformula_msgs::msg::ObjectInfoMultiArray pub_msg;
    pub_msg.header = header;

    for (const auto& tracked_object : tracked_objects_) {
        const auto bottom_left_in_odom = tf2::Vector3(tracked_object.getLeftX(), tracked_object.getLeftY(), 0.0);
        const auto bottom_right_in_odom = tf2::Vector3(tracked_object.getRightX(), tracked_object.getRightY(), 0.0);
        const auto bottom_left_in_vehicle = vehicle_T_odom * bottom_left_in_odom;
        const auto bottom_right_in_vehicle = vehicle_T_odom * bottom_right_in_odom;
        auto& object_info = pub_msg.objects.emplace_back();
        object_info.x = (bottom_left_in_vehicle.x() + bottom_right_in_vehicle.x()) * 0.5;
        object_info.y = (bottom_left_in_vehicle.y() + bottom_right_in_vehicle.y()) * 0.5;
        object_info.width = std::abs(bottom_left_in_vehicle.y() - bottom_right_in_vehicle.y());
        object_info.id = tracked_object.getId();
        object_info.confidence = tracked_object.getConfidence();
    };
    object_pub_->publish(pub_msg);
    RCLCPP_INFO_ONCE(this->get_logger(), "Publish Object Info !");
}

}  // namespace aiformula
