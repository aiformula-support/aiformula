#include "object_publisher/tracked_object.hpp"

#include "common_cpp/get_ros_parameter.hpp"

namespace aiformula {

std::optional<rclcpp::Node*> TrackedObject::node_ptr_;
std::optional<double> TrackedObject::process_noise_variance_;
std::optional<double> TrackedObject::measurement_noise_variance_;
std::optional<double> TrackedObject::initial_error_covariance_;
std::optional<double> TrackedObject::expiration_duration_;

void TrackedObject::initStaticMembers(rclcpp::Node* const node_ptr) {
    node_ptr_ = node_ptr;

    // From 'tracked_object.yaml'
    process_noise_variance_ = getRosParameter<double>(node_ptr_.value(), "process_noise_variance");
    measurement_noise_variance_ = getRosParameter<double>(node_ptr_.value(), "measurement_noise_variance");
    initial_error_covariance_ = getRosParameter<double>(node_ptr_.value(), "initial_error_covariance");
    expiration_duration_ = getRosParameter<double>(node_ptr_.value(), "expiration_duration");
}

void TrackedObject::printStaticMembers() {
    std::ostringstream log_stream;
    const auto& formatter = createFormatter(30);
    log_stream << "\n[" << __func__ << " (TrackedObject)] " << std::string(33, '=') << "\n"
               << "(tracked_object.yaml)\n"
               << std::fixed << std::setprecision(1) << std::scientific << formatter("process_noise_variance_")
               << process_noise_variance_.value() << "\n"
               << formatter("measurement_noise_variance_") << measurement_noise_variance_.value() << "\n"
               << formatter("initial_error_covariance_") << initial_error_covariance_.value() << "\n"
               << std::fixed << formatter("expiration_duration_") << expiration_duration_.value() << " [sec]\n"
               << std::string(70, '=') << "\n";
    RCLCPP_INFO(node_ptr_.value()->get_logger(), "%s", log_stream.str().c_str());
}

TrackedObject::TrackedObject(const int& id, const float& left_x, const float& left_y, const float& right_x,
                             const float& right_y, const double& timestamp)
    : kf_(cv::KalmanFilter(4, 4, 0)), id_(id), last_seen_time_(timestamp), confidence_(1.0) {
    cv::setIdentity(kf_.transitionMatrix);
    cv::setIdentity(kf_.measurementMatrix);

    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(process_noise_variance_.value()));
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(measurement_noise_variance_.value()));
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(initial_error_covariance_.value()));

    kf_.statePost.at<float>(0) = left_x;
    kf_.statePost.at<float>(1) = left_y;
    kf_.statePost.at<float>(2) = right_x;
    kf_.statePost.at<float>(3) = right_y;
}

float TrackedObject::computeDistanceSquared(const float& x_in, const float& y_in) const {
    const auto obj_x = getCenterX();
    const auto obj_y = getCenterY();
    return (obj_x - x_in) * (obj_x - x_in) + (obj_y - y_in) * (obj_y - y_in);
}

void TrackedObject::update(const float& left_x, const float& left_y, const float& right_x, const float& right_y,
                           const double& current_time) {
    cv::Mat measurement = (cv::Mat_<float>(4, 1) << left_x, left_y, right_x, right_y);
    kf_.predict();
    kf_.correct(measurement);
    last_seen_time_ = current_time;
}

bool TrackedObject::isExpired(const double& current_time) {
    if (current_time < last_seen_time_) return true;  // Loop playback detected during rosbag play mode..

    confidence_ = 1.0 - (current_time - last_seen_time_) / expiration_duration_.value();
    return confidence_ < 0.0;
}

}  // namespace aiformula
