#include "common_cpp/to_geometry_msgs.hpp"

namespace aiformula {

geometry_msgs::msg::Point toPointMsg(const double& x, const double& y, const double& z) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::msg::Quaternion toQuaternionMsg(const double& roll, const double& pitch, const double& yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
}

geometry_msgs::msg::Pose toPoseMsg(const tf2::Vector3& pos, const tf2::Vector3& rot) {
    geometry_msgs::msg::Pose pose;
    pose.position = toPointMsg(pos.x(), pos.y(), pos.z());
    pose.orientation = toQuaternionMsg(rot.x(), rot.y(), rot.z());
    return pose;
}

geometry_msgs::msg::Pose toPoseMsg(const tf2::Vector3& pos) { return toPoseMsg(pos, tf2::Vector3(0.0, 0.0, 0.0)); }

geometry_msgs::msg::Vector3 toVector3Msg(const double& x, const double& y, const double& z) {
    geometry_msgs::msg::Vector3 vec;
    vec.x = x;
    vec.y = y;
    vec.z = z;
    return vec;
}

geometry_msgs::msg::Vector3 toVector3Msg(const geometry_msgs::msg::Point& point) {
    geometry_msgs::msg::Vector3 vec;
    vec.x = point.x;
    vec.y = point.y;
    vec.z = point.z;
    return vec;
}

}  // namespace aiformula
