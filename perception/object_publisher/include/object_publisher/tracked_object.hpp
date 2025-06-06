#ifndef TRACKED_OBJECT_HPP
#define TRACKED_OBJECT_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// C++
#include <optional>

// OpenCV
#include <opencv2/opencv.hpp>

namespace aiformula {

class TrackedObject {
public:
    static void initStaticMembers(rclcpp::Node* const node_ptr);
    static void printStaticMembers();
    TrackedObject(const int& id, const float& left_x, const float& left_y, const float& right_x, const float& right_y,
                  const double& timestamp);
    ~TrackedObject() = default;
    float computeDistanceSquared(const float& x_in, const float& y_in) const;
    void update(const float& left_x, const float& left_y, const float& right_x, const float& right_y,
                const double& current_time);
    bool isExpired(const double& current_time);

    float getId() const { return id_; }
    float getConfidence() const { return confidence_; }
    float getLeftX() const { return kf_.statePost.at<float>(0); }
    float getLeftY() const { return kf_.statePost.at<float>(1); }
    float getRightX() const { return kf_.statePost.at<float>(2); }
    float getRightY() const { return kf_.statePost.at<float>(3); }
    float getCenterX() const { return (getLeftX() + getRightX()) * 0.5; }
    float getCenterY() const { return (getLeftY() + getRightY()) * 0.5; }

private:
    static std::optional<rclcpp::Node*> node_ptr_;
    static std::optional<double> process_noise_variance_;
    static std::optional<double> measurement_noise_variance_;
    static std::optional<double> initial_error_covariance_;
    static std::optional<double> expiration_duration_;

    cv::KalmanFilter kf_;
    unsigned int id_;
    double last_seen_time_;
    double confidence_;
};

}  // namespace aiformula

#endif  // TRACKED_OBJECT_HPP
