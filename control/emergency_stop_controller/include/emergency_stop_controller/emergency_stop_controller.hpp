#ifndef EMERGENCY_STOP_CONTROLLER_HPP
#define EMERGENCY_STOP_CONTROLLER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace aiformula {

class EmergencyStopController : public rclcpp::Node {
public:
    explicit EmergencyStopController();
    ~EmergencyStopController() = default;

private:
    struct InitParams {
        int publish_twist_timer_loop_duration;  // [msec]
        InitParams() : publish_twist_timer_loop_duration(100) {}
    };
    static const int EMERGENCY_CAN_ID = 1808;

    void getRosParams(InitParams& init_params);
    void initValues(const InitParams& init_params);
    void printParam(const InitParams& init_params) const;
    void canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg);
    void publishTwistCallback();

    rclcpp::TimerBase::SharedPtr publish_twist_timer_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    bool is_emergency_;
    geometry_msgs::msg::Twist stop_twist_;  // Zero twist used for emergency stop (default-initialized)
};

}  // namespace aiformula

#endif  // EMERGENCY_STOP_CONTROLLER_HPP
