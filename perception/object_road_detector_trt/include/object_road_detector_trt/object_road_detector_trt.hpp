#ifndef OBJECT_ROAD_DETECTOR_TRT_OBJECT_ROAD_DETECTOR_TRT_HPP
#define OBJECT_ROAD_DETECTOR_TRT_OBJECT_ROAD_DETECTOR_TRT_HPP

#include <cuda_runtime_api.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "aiformula_msgs/msg/rect_multi_array.hpp"
#include "object_road_detector_trt/cuda_util.hpp"
#include "object_road_detector_trt/kernels.hpp"
#include "object_road_detector_trt/trt_engine.hpp"

namespace aiformula {

/**
 * @brief Runs the YOLOP lane and object heads as a TensorRT engine, with pre- and post-processing
 * on the GPU.
 *
 * Composed into the same container as the ZED wrapper, so the camera image never leaves the
 * process. Publishes the same lane mask and bounding box topics as the Python
 * `object_road_detector`, which keeps `lane_line_publisher` and `object_publisher` unchanged.
 */
class ObjectRoadDetectorTrt : public rclcpp::Node {
public:
    explicit ObjectRoadDetectorTrt(const rclcpp::NodeOptions& options);
    ~ObjectRoadDetectorTrt();

private:
    /// How the source frame is reduced to the fixed-size engine input.
    enum class InputMode {
        /// Crop a rectangle at the sensor scale, so an object keeps the pixel size the detect head
        /// anchors were trained for. Distant objects gain resolution as the camera does.
        Roi,
        /// Scale the whole frame down with letterbox padding, reproducing the Python behaviour.
        Letterbox,
    };

    void getRosParams();
    void initEngine();
    void initTopics();
    void printParams() const;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    /// Recomputes the source rectangle and reallocates the buffers when the camera size changes.
    void configureFor(int source_width, int source_height);
    void publishRects(const std_msgs::msg::Header& header) const;
    sensor_msgs::msg::Image::UniquePtr buildMaskMessage(const std_msgs::msg::Header& header) const;
    void publishAnnotatedImage(const sensor_msgs::msg::Image& source,
                               const sensor_msgs::msg::Image& mask) const;
    void reportRate(double callback_ms);

    // Parameters
    std::string engine_path_;
    InputMode input_mode_{InputMode::Roi};
    int roi_x_offset_{-1};
    int roi_y_offset_{-1};
    float horizon_ratio_{0.55F};
    bool swap_red_blue_{false};
    float score_threshold_{0.0F};
    float mean_[3]{};
    float stdev_[3]{};
    bool debug_{false};
    bool enable_profiling_{false};
    double profiling_interval_sec_{5.0};

    // Engine geometry
    int engine_width_{0};
    int engine_height_{0};
    int max_detections_{0};

    // Source geometry, resolved on the first frame
    int source_width_{0};
    int source_height_{0};
    int origin_x_{0};
    int origin_y_{0};
    int extent_width_{0};
    int extent_height_{0};
    InputTransform transform_{1.0F, 0.0F, 0.0F};

    std::unique_ptr<TrtEngine> engine_;
    cudaStream_t stream_{nullptr};

    DeviceBuffer device_source_;
    DeviceBuffer device_input_;
    DeviceBuffer device_mask_;
    DeviceBuffer device_num_detections_;
    DeviceBuffer device_boxes_;
    DeviceBuffer device_scores_;
    DeviceBuffer device_classes_;
    DeviceBuffer device_lane_score_;

    HostPinnedBuffer host_num_detections_;
    HostPinnedBuffer host_boxes_;
    HostPinnedBuffer host_scores_;
    HostPinnedBuffer host_mask_;

    int frame_count_{0};
    double accumulated_ms_{0.0};
    std::chrono::steady_clock::time_point window_start_{std::chrono::steady_clock::now()};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_image_pub_;
    rclcpp::Publisher<aiformula_msgs::msg::RectMultiArray>::SharedPtr rects_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;
};

}  // namespace aiformula

#endif  // OBJECT_ROAD_DETECTOR_TRT_OBJECT_ROAD_DETECTOR_TRT_HPP
