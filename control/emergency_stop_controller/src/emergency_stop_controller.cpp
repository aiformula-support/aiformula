#include "emergency_stop_controller/emergency_stop_controller.hpp"

#include "common_cpp/get_ros_parameter.hpp"
#include "common_cpp/util.hpp"

namespace aiformula {

EmergencyStopController::EmergencyStopController()
    : Node("emergency_stop_controller"),
      is_emergency_(false),
      stop_twist_()  // All fields set to 0.0 by default constructor
{
    InitParams init_params;
    getRosParams(init_params);
    initValues(init_params);
    printParam(init_params);
}

void EmergencyStopController::getRosParams(InitParams& init_params) {
    // From 'emergency_stop_controller.yaml'
    init_params.publish_twist_timer_loop_duration = getRosParameter<int>(this, "publish_twist_timer_loop_duration");
}

void EmergencyStopController::initValues(const InitParams& init_params) {
    // Timer
    publish_twist_timer_ = create_wall_timer(std::chrono::milliseconds(init_params.publish_twist_timer_loop_duration),
                                             std::bind(&EmergencyStopController::publishTwistCallback, this));

    // Subscriber & Publisher
    const int buffer_size = 10;
    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "sub_can", buffer_size, std::bind(&EmergencyStopController::canFrameCallback, this, std::placeholders::_1));
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("pub_twist", buffer_size);
}

void EmergencyStopController::printParam(const InitParams& init_params) const {
    std::ostringstream log_stream;
    const auto formatter = createFormatter(33);
    log_stream << "\n[" << __func__ << "] " << std::string(57, '=') << "\n"
               << "(emergency_stop_controller.yaml)\n"
               << formatter("publish_twist_timer_loop_duration") << init_params.publish_twist_timer_loop_duration
               << " [msec]\n"
               << std::string(70, '=') << "\n";
    RCLCPP_INFO(this->get_logger(), "%s", log_stream.str().c_str());
}

void EmergencyStopController::canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Can Frame !");
    if (msg->id != this->EMERGENCY_CAN_ID) return;

    if (msg->dlc != 4) {
        RCLCPP_WARN(this->get_logger(), "Invalid DLC: expected 4, got %d (id=0x%X)", msg->dlc, msg->id);
        return;
    }

    if (msg->data[0] == 8 && msg->data[2] == 8) {
        is_emergency_ = true;
    } else {
        is_emergency_ = false;
    }
}

void EmergencyStopController::publishTwistCallback() {
    if (is_emergency_) {
        twist_pub_->publish(stop_twist_);
        RCLCPP_INFO_ONCE(this->get_logger(), "Publish Twist !");
    }
}

}  // namespace aiformula
