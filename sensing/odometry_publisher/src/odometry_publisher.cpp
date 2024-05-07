#include "odometry_publisher/odometry_publisher.hpp"

namespace aiformula {

OdometryPublisher::OdometryPublisher(const std::string& node_name)
    : Node(node_name),
      prev_time_(0.0),
      pos_(toPointMsg(0.0, 0.0, 0.0)),
      yaw_angle_(0.0),
      vehicle_linear_velocity_(toVector3Msg(0.0, 0.0, 0.0)),
      yaw_rate_(0.0) {
    getRosParams();
    initValues();
    printParam();
}

void OdometryPublisher::getRosParams() {
    // From 'launch file'
    odom_frame_id_ = getRosParameter<std::string>(this, "odom_frame_id");
    vehicle_frame_id_ = getRosParameter<std::string>(this, "vehicle_frame_id");
}

void OdometryPublisher::initValues() {
    const int buffer_size = 10;
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("pub_odometry", buffer_size);
    odometry_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void OdometryPublisher::printParam() const {
    RCLCPP_INFO(this->get_logger(), "[%s] ===============", __func__);
    RCLCPP_INFO(this->get_logger(), "(launch)");
    RCLCPP_INFO(this->get_logger(), "  odom_frame_id_    = %s", odom_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  vehicle_frame_id_ = %s", vehicle_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "============================\n");
}

double OdometryPublisher::calcTimeDelta(const builtin_interfaces::msg::Time& msg_time) {
    if (!prev_time_) {
        prev_time_ = toTimeStampDouble(msg_time);
        return 0.0;
    }
    const double current_time = toTimeStampDouble(msg_time);
    const double dt = current_time - prev_time_;
    prev_time_ = current_time;
    return dt;
}

void OdometryPublisher::updatePosition(const double& vehicle_linear_velocity, const double& dt) {
    vehicle_linear_velocity_.x = vehicle_linear_velocity * cos(yaw_angle_);
    vehicle_linear_velocity_.y = vehicle_linear_velocity * sin(yaw_angle_);
    pos_.x += vehicle_linear_velocity_.x * dt;
    pos_.y += vehicle_linear_velocity_.y * dt;
}

nav_msgs::msg::Odometry OdometryPublisher::createOdometryMsg(const builtin_interfaces::msg::Time& msg_time) const {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = msg_time;
    odometry.header.frame_id = odom_frame_id_;
    odometry.child_frame_id = vehicle_frame_id_;
    odometry.pose.pose.position = pos_;
    odometry.pose.pose.orientation = toQuaternionMsg(0.0, 0.0, yaw_angle_);
    odometry.twist.twist.linear = vehicle_linear_velocity_;
    odometry.twist.twist.angular = toVector3Msg(0.0, 0.0, yaw_rate_);
    return odometry;
}

void OdometryPublisher::broadcastTf(const nav_msgs::msg::Odometry& odom) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header = odom.header;
    ts.child_frame_id = odom.child_frame_id;
    ts.transform.translation = toVector3Msg(odom.pose.pose.position);
    ts.transform.rotation = odom.pose.pose.orientation;
    odometry_br_->sendTransform(ts);
}

}  // namespace aiformula
