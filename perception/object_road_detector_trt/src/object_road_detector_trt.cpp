#include "object_road_detector_trt/object_road_detector_trt.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "common_cpp/get_ros_parameter.hpp"

namespace aiformula {
namespace {

constexpr float kLetterboxPadValue = 114.0F / 255.0F;
constexpr char kInputName[] = "images";
constexpr char kNumDetectionsName[] = "num_detections";
constexpr char kBoxesName[] = "detection_boxes";
constexpr char kScoresName[] = "detection_scores";
constexpr char kClassesName[] = "detection_classes";
constexpr char kLaneScoreName[] = "lane_score";

int encodingChannels(const std::string& encoding) {
    if (encoding == "bgra8" || encoding == "rgba8") return 4;
    if (encoding == "bgr8" || encoding == "rgb8") return 3;
    return 0;
}

bool encodingIsRgb(const std::string& encoding) { return encoding == "rgb8" || encoding == "rgba8"; }

int clampOrigin(const int origin, const int extent, const int limit) {
    return std::max(0, std::min(origin, limit - extent));
}

}  // namespace

ObjectRoadDetectorTrt::ObjectRoadDetectorTrt(const rclcpp::NodeOptions& options)
    : Node("object_road_detector", options) {
    getRosParams();
    CUDA_CHECK(cudaStreamCreate(&stream_));
    initEngine();
    initTopics();
    printParams();
}

ObjectRoadDetectorTrt::~ObjectRoadDetectorTrt() {
    if (stream_) {
        cudaStreamDestroy(stream_);
        stream_ = nullptr;
    }
}

void ObjectRoadDetectorTrt::getRosParams() {
    engine_path_ = getRosParameter<std::string>(this, "engine_path");
    const auto mode = getRosParameter<std::string>(this, "input_mode");
    if (mode == "roi") {
        input_mode_ = InputMode::Roi;
    } else if (mode == "letterbox") {
        input_mode_ = InputMode::Letterbox;
    } else {
        throw std::runtime_error("input_mode must be 'roi' or 'letterbox', got '" + mode + "'");
    }
    roi_x_offset_ = getRosParameter<int>(this, "roi_x_offset");
    roi_y_offset_ = getRosParameter<int>(this, "roi_y_offset");
    horizon_ratio_ = static_cast<float>(getRosParameter<double>(this, "horizon_ratio"));
    swap_red_blue_ = getRosParameter<bool>(this, "swap_red_blue");
    score_threshold_ = static_cast<float>(getRosParameter<double>(this, "score_threshold"));
    const auto mean = getRosParameter<std::vector<double>>(this, "normalization.mean");
    const auto stdev = getRosParameter<std::vector<double>>(this, "normalization.standard_deviation");
    if (mean.size() != 3 || stdev.size() != 3) {
        throw std::runtime_error("normalization.mean and standard_deviation must each have 3 values");
    }
    for (int channel = 0; channel < 3; ++channel) {
        mean_[channel] = static_cast<float>(mean[channel]);
        stdev_[channel] = static_cast<float>(stdev[channel]);
    }
    debug_ = getRosParameter<bool>(this, "debug");
    enable_profiling_ = getRosParameter<bool>(this, "profiling.enable");
    profiling_interval_sec_ = getRosParameter<double>(this, "profiling.report_interval_sec");
}

void ObjectRoadDetectorTrt::initEngine() {
    engine_ = std::make_unique<TrtEngine>(engine_path_, get_logger());
    RCLCPP_INFO(get_logger(), "Loaded TensorRT engine %s%s", engine_path_.c_str(), engine_->describe().c_str());

    const auto input_shape = engine_->tensorShape(kInputName);
    if (input_shape.size() != 4) {
        throw std::runtime_error("engine input 'images' must be NCHW");
    }
    engine_height_ = static_cast<int>(input_shape[2]);
    engine_width_ = static_cast<int>(input_shape[3]);
    const auto box_shape = engine_->tensorShape(kBoxesName);
    max_detections_ = static_cast<int>(box_shape.size() >= 2 ? box_shape[1] : 0);

    device_input_.reserve(engine_->tensorByteSize(kInputName));
    device_num_detections_.reserve(engine_->tensorByteSize(kNumDetectionsName));
    device_boxes_.reserve(engine_->tensorByteSize(kBoxesName));
    device_scores_.reserve(engine_->tensorByteSize(kScoresName));
    device_classes_.reserve(engine_->tensorByteSize(kClassesName));
    device_lane_score_.reserve(engine_->tensorByteSize(kLaneScoreName));

    host_num_detections_.reserve(engine_->tensorByteSize(kNumDetectionsName));
    host_boxes_.reserve(engine_->tensorByteSize(kBoxesName));
    host_scores_.reserve(engine_->tensorByteSize(kScoresName));

    engine_->bind(kInputName, device_input_.data());
    engine_->bind(kNumDetectionsName, device_num_detections_.data());
    engine_->bind(kBoxesName, device_boxes_.data());
    engine_->bind(kScoresName, device_scores_.data());
    engine_->bind(kClassesName, device_classes_.data());
    engine_->bind(kLaneScoreName, device_lane_score_.data());
}

void ObjectRoadDetectorTrt::initTopics() {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "sub_image", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&ObjectRoadDetectorTrt::imageCallback, this, std::placeholders::_1));
    constexpr int kQueueSize = 10;
    mask_image_pub_ = create_publisher<sensor_msgs::msg::Image>("pub_mask_image", kQueueSize);
    rects_pub_ = create_publisher<aiformula_msgs::msg::RectMultiArray>("pub_bbox", kQueueSize);
    if (debug_) {
        annotated_image_pub_ = create_publisher<sensor_msgs::msg::Image>("pub_annotated_image", kQueueSize);
    }
}

void ObjectRoadDetectorTrt::printParams() const {
    RCLCPP_INFO(get_logger(), "[object_road_detector_trt]");
    RCLCPP_INFO(get_logger(), "  engine_path     : %s", engine_path_.c_str());
    RCLCPP_INFO(get_logger(), "  engine input    : %dx%d", engine_width_, engine_height_);
    RCLCPP_INFO(get_logger(), "  input_mode      : %s", input_mode_ == InputMode::Roi ? "roi" : "letterbox");
    RCLCPP_INFO(get_logger(), "  roi offset      : x=%d y=%d (negative = auto)", roi_x_offset_, roi_y_offset_);
    RCLCPP_INFO(get_logger(), "  horizon_ratio   : %.3f", horizon_ratio_);
    RCLCPP_INFO(get_logger(), "  debug           : %s", debug_ ? "true" : "false");
}

void ObjectRoadDetectorTrt::configureFor(const int source_width, const int source_height) {
    source_width_ = source_width;
    source_height_ = source_height;

    if (input_mode_ == InputMode::Roi) {
        extent_width_ = std::min(engine_width_, source_width_);
        extent_height_ = std::min(engine_height_, source_height_);
        origin_x_ = roi_x_offset_ >= 0 ? clampOrigin(roi_x_offset_, extent_width_, source_width_)
                                       : (source_width_ - extent_width_) / 2;
        const int auto_y =
            static_cast<int>(std::lround(static_cast<double>(horizon_ratio_) * source_height_ - extent_height_ / 2.0));
        origin_y_ = roi_y_offset_ >= 0 ? clampOrigin(roi_y_offset_, extent_height_, source_height_)
                                       : clampOrigin(auto_y, extent_height_, source_height_);
        transform_.scale = 1.0F;
        transform_.pad_x = 0.5F * static_cast<float>(engine_width_ - extent_width_);
        transform_.pad_y = 0.5F * static_cast<float>(engine_height_ - extent_height_);
    } else {
        origin_x_ = 0;
        origin_y_ = 0;
        extent_width_ = source_width_;
        extent_height_ = source_height_;
        const float scale = std::min(static_cast<float>(engine_width_) / static_cast<float>(source_width_),
                                     static_cast<float>(engine_height_) / static_cast<float>(source_height_));
        transform_.scale = scale;
        transform_.pad_x = 0.5F * (static_cast<float>(engine_width_) - static_cast<float>(source_width_) * scale);
        transform_.pad_y = 0.5F * (static_cast<float>(engine_height_) - static_cast<float>(source_height_) * scale);
    }

    const std::size_t mask_bytes = static_cast<std::size_t>(source_width_) * static_cast<std::size_t>(source_height_);
    device_mask_.reserve(mask_bytes);
    host_mask_.reserve(mask_bytes);

    RCLCPP_INFO(get_logger(),
                "Configured for %dx%d source: window origin=(%d,%d) extent=%dx%d scale=%.4f pad=(%.1f,%.1f)",
                source_width_, source_height_, origin_x_, origin_y_, extent_width_, extent_height_, transform_.scale,
                transform_.pad_x, transform_.pad_y);
}

void ObjectRoadDetectorTrt::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    const auto start = std::chrono::steady_clock::now();
    const int channels = encodingChannels(msg->encoding);
    if (channels == 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported encoding '%s'", msg->encoding.c_str());
        return;
    }
    if (static_cast<int>(msg->width) != source_width_ || static_cast<int>(msg->height) != source_height_) {
        configureFor(static_cast<int>(msg->width), static_cast<int>(msg->height));
    }

    const std::size_t source_bytes =
        static_cast<std::size_t>(extent_width_) * static_cast<std::size_t>(extent_height_) * channels;
    device_source_.reserve(source_bytes);

    const std::size_t src_pitch = msg->step;
    const std::uint8_t* const src_origin =
        msg->data.data() + static_cast<std::size_t>(origin_y_) * src_pitch + origin_x_ * channels;
    const std::size_t tight_pitch = static_cast<std::size_t>(extent_width_) * channels;
    CUDA_CHECK(cudaMemcpy2DAsync(device_source_.data(), tight_pitch, src_origin, src_pitch, tight_pitch, extent_height_,
                                 cudaMemcpyHostToDevice, stream_));

    const bool swap_red_blue = swap_red_blue_ || encodingIsRgb(msg->encoding);
    launchPreprocess(device_source_.data<std::uint8_t>(), static_cast<int>(tight_pitch), channels, extent_width_,
                     extent_height_, device_input_.data<float>(), engine_width_, engine_height_, transform_, mean_,
                     stdev_, kLetterboxPadValue, swap_red_blue, stream_);

    engine_->enqueue(stream_);

    const std::size_t mask_bytes = static_cast<std::size_t>(source_width_) * static_cast<std::size_t>(source_height_);
    CUDA_CHECK(cudaMemsetAsync(device_mask_.data(), 0, mask_bytes, stream_));
    std::uint8_t* const mask_roi =
        device_mask_.data<std::uint8_t>() + static_cast<std::size_t>(origin_y_) * source_width_ + origin_x_;
    const auto lane_shape = engine_->tensorShape(kLaneScoreName);
    const int score_height = static_cast<int>(lane_shape.size() >= 4 ? lane_shape[2] : engine_height_);
    const int score_width = static_cast<int>(lane_shape.size() >= 4 ? lane_shape[3] : engine_width_);
    launchLaneMask(device_lane_score_.data<float>(), score_width, score_height, mask_roi, source_width_, extent_width_,
                   extent_height_, transform_, stream_);

    CUDA_CHECK(cudaMemcpyAsync(host_num_detections_.data(), device_num_detections_.data(),
                               engine_->tensorByteSize(kNumDetectionsName), cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaMemcpyAsync(host_boxes_.data(), device_boxes_.data(), engine_->tensorByteSize(kBoxesName),
                               cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaMemcpyAsync(host_scores_.data(), device_scores_.data(), engine_->tensorByteSize(kScoresName),
                               cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaMemcpyAsync(host_mask_.data(), device_mask_.data(), mask_bytes, cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));

    publishRects(msg->header);
    auto mask_msg = buildMaskMessage(msg->header);
    if (annotated_image_pub_) {
        publishAnnotatedImage(*msg, *mask_msg);
    }
    mask_image_pub_->publish(std::move(mask_msg));

    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    reportRate(elapsed_ms);
}

void ObjectRoadDetectorTrt::publishRects(const std_msgs::msg::Header& header) const {
    const int count = std::max(0, std::min(max_detections_, host_num_detections_.data<int>()[0]));
    const float* const boxes = host_boxes_.data<float>();
    const float* const scores = host_scores_.data<float>();
    aiformula_msgs::msg::RectMultiArray bbox_msg;
    bbox_msg.header = header;
    bbox_msg.rects.reserve(count);
    for (int index = 0; index < count; ++index) {
        if (scores[index] < score_threshold_) continue;
        const float* const box = boxes + index * 4;
        aiformula_msgs::msg::Rect rect;
        const float x1 = (box[0] - transform_.pad_x) / transform_.scale + static_cast<float>(origin_x_);
        const float y1 = (box[1] - transform_.pad_y) / transform_.scale + static_cast<float>(origin_y_);
        const float x2 = (box[2] - transform_.pad_x) / transform_.scale + static_cast<float>(origin_x_);
        const float y2 = (box[3] - transform_.pad_y) / transform_.scale + static_cast<float>(origin_y_);
        rect.x = x1;
        rect.y = y1;
        rect.width = x2 - x1;
        rect.height = y2 - y1;
        bbox_msg.rects.push_back(rect);
    }
    rects_pub_->publish(bbox_msg);
}

sensor_msgs::msg::Image::UniquePtr ObjectRoadDetectorTrt::buildMaskMessage(
    const std_msgs::msg::Header& header) const {
    auto mask_msg = std::make_unique<sensor_msgs::msg::Image>();
    mask_msg->header = header;
    mask_msg->height = source_height_;
    mask_msg->width = source_width_;
    mask_msg->encoding = "mono8";
    mask_msg->is_bigendian = 0;
    mask_msg->step = source_width_;
    const std::size_t bytes = static_cast<std::size_t>(source_width_) * static_cast<std::size_t>(source_height_);
    mask_msg->data.resize(bytes);
    std::memcpy(mask_msg->data.data(), host_mask_.data(), bytes);
    return mask_msg;
}

void ObjectRoadDetectorTrt::publishAnnotatedImage(const sensor_msgs::msg::Image& source,
                                                  const sensor_msgs::msg::Image& mask) const {
    const int channels = encodingChannels(source.encoding);
    const int cv_type = channels == 4 ? CV_8UC4 : CV_8UC3;
    cv::Mat wrapped(source_height_, source_width_, cv_type, const_cast<std::uint8_t*>(source.data.data()), source.step);
    cv::Mat bgr;
    if (channels == 4) {
        cv::cvtColor(wrapped, bgr, encodingIsRgb(source.encoding) ? cv::COLOR_RGBA2BGR : cv::COLOR_BGRA2BGR);
    } else if (encodingIsRgb(source.encoding)) {
        cv::cvtColor(wrapped, bgr, cv::COLOR_RGB2BGR);
    } else {
        bgr = wrapped.clone();
    }

    // Lane overlay (same green tint as the Python detector).
    cv::Mat mask_mat(source_height_, source_width_, CV_8UC1, const_cast<std::uint8_t*>(mask.data.data()), mask.step);
    cv::Mat overlay = bgr.clone();
    overlay.setTo(cv::Scalar(0, 255, 0), mask_mat);
    cv::addWeighted(overlay, 0.5, bgr, 0.5, 0.0, bgr);

    // Bounding boxes in source-image coordinates (EfficientNMS already returns corners).
    const int count = std::max(0, std::min(max_detections_, host_num_detections_.data<int>()[0]));
    const float* const boxes = host_boxes_.data<float>();
    const float* const scores = host_scores_.data<float>();
    const int thickness = std::max(static_cast<int>(source_height_ * 0.002), 1);
    for (int index = 0; index < count; ++index) {
        if (scores[index] < score_threshold_) continue;
        const float* const box = boxes + index * 4;
        const int x1 = static_cast<int>((box[0] - transform_.pad_x) / transform_.scale + static_cast<float>(origin_x_));
        const int y1 = static_cast<int>((box[1] - transform_.pad_y) / transform_.scale + static_cast<float>(origin_y_));
        const int x2 = static_cast<int>((box[2] - transform_.pad_x) / transform_.scale + static_cast<float>(origin_x_));
        const int y2 = static_cast<int>((box[3] - transform_.pad_y) / transform_.scale + static_cast<float>(origin_y_));
        cv::rectangle(bgr, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), thickness, cv::LINE_AA);
    }

    sensor_msgs::msg::Image annotated;
    annotated.header = source.header;
    annotated.height = source_height_;
    annotated.width = source_width_;
    annotated.encoding = "bgr8";
    annotated.is_bigendian = 0;
    annotated.step = source_width_ * 3;
    annotated.data.resize(static_cast<std::size_t>(annotated.step) * source_height_);
    for (int row = 0; row < source_height_; ++row) {
        std::memcpy(annotated.data.data() + static_cast<std::size_t>(row) * annotated.step, bgr.ptr(row),
                    annotated.step);
    }
    annotated_image_pub_->publish(annotated);
}

void ObjectRoadDetectorTrt::reportRate(const double callback_ms) {
    if (!enable_profiling_) return;
    ++frame_count_;
    accumulated_ms_ += callback_ms;
    const double window_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - window_start_).count();
    if (window_sec < profiling_interval_sec_) return;
    RCLCPP_INFO(get_logger(), "[profile] %.2f FPS over %.1fs, %.2f ms/frame", frame_count_ / window_sec, window_sec,
                accumulated_ms_ / frame_count_);
    frame_count_ = 0;
    accumulated_ms_ = 0.0;
    window_start_ = std::chrono::steady_clock::now();
}

}  // namespace aiformula

RCLCPP_COMPONENTS_REGISTER_NODE(aiformula::ObjectRoadDetectorTrt)
