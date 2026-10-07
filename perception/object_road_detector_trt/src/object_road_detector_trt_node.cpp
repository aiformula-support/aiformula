#include "object_road_detector_trt/object_road_detector_trt.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false);
    rclcpp::spin(std::make_shared<aiformula::ObjectRoadDetectorTrt>(options));
    rclcpp::shutdown();
    return 0;
}
