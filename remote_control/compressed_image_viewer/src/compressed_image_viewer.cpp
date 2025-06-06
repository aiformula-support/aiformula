#include "compressed_image_viewer/compressed_image_viewer.hpp"

namespace aiformula {

CompressedImageViewer::CompressedImageViewer()
    : Node("compressed_image_viewer"),
      display_full_screen_(true),
      target_screen_idx_(0),
      window_width_ratio_(0.5),
      window_position_ratio_(cv::Point2d(0.0, 0.0)),
      window_name_("AI Formula Pilot") {}

void CompressedImageViewer::setup() {
    getRosParams();
    initValues();
    printParam();
}

void CompressedImageViewer::getRosParams() {
    // compressed_image_viewer.yaml
    display_full_screen_ = getRosParameter<bool>(this, "display_full_screen");

    if (!display_full_screen_) {
        target_screen_idx_ = getRosParameter<int>(this, "target_screen_idx");
        window_width_ratio_ = getRosParameter<double>(this, "window.width_ratio");
        window_position_ratio_.x = getRosParameter<double>(this, "window.position_ratio.x");
        window_position_ratio_.y = getRosParameter<double>(this, "window.position_ratio.y");

        // Check the range of parameter values.
        try {
            if (target_screen_idx_ < 0) throw ScreenIndexException(target_screen_idx_);
            if (window_width_ratio_ <= 0.0) throw WindowWidthRatioException(window_width_ratio_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[Exception] " << e.what());
            rclcpp::shutdown();
            exit(1);
        }
    }
}

void CompressedImageViewer::initValues() {
    // Subscriber
    const int buffer_size = 10;
    compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "sub_compressed_image", buffer_size,
        std::bind(&CompressedImageViewer::compressedImageCallback, this, std::placeholders::_1));

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    if (display_full_screen_) {
        cv::setWindowProperty(window_name_, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        /* In older versions of OpenCV, 'cv::setWindowProperty()' makes the window full screen,
           but the displayed image is not scaled to full screen.
           By executing 'cv::resizeWindow()', even just as a formality,
           the displayed image is scaled to full screen (the reason is unknown). */
        cv::resizeWindow(window_name_, cv::Size());
    } else {
        GdkRectangle monitor_geometry;
        getScreenInfo(monitor_geometry);
        setWindowSizeAndPosition(monitor_geometry);
    }
}

void CompressedImageViewer::getScreenInfo(GdkRectangle& monitor_geometry) const {
    try {
#if GDK_VERSION == 2
        gdk_init(NULL, NULL);
        GdkScreen* screen = gdk_screen_get_default();
        const int num_monitors = gdk_screen_get_n_monitors(screen);
        if (target_screen_idx_ >= num_monitors) {
            throw ScreenIndexException(target_screen_idx_, num_monitors);
        }
        gdk_screen_get_monitor_geometry(screen, target_screen_idx_, &monitor_geometry);
#elif GDK_VERSION == 3
        gdk_init(NULL, NULL);
        GdkDisplay* display = gdk_display_get_default();
        GdkMonitor* monitor = gdk_display_get_monitor(display, target_screen_idx_);
        if (!monitor) throw ScreenIndexException(target_screen_idx_, gdk_display_get_n_monitors(display));
        gdk_monitor_get_geometry(monitor, &monitor_geometry);
#else
        if (GDK_VERSION)
            throw GdkVersionException(GDK_VERSION);
        else
            throw GdkVersionException();
#endif
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[Exception] " << e.what());
        rclcpp::shutdown();
        exit(1);
    }
}

void CompressedImageViewer::setWindowSizeAndPosition(const GdkRectangle& monitor_geometry) {
    // Get the image size.
    sensor_msgs::msg::CompressedImage msg;
    if (!rclcpp::wait_for_message(msg, shared_from_this(), "sub_compressed_image", std::chrono::milliseconds(3000))) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't receive a CompressedImage topic.");
        rclcpp::shutdown();
        exit(1);
    }
    const auto image_size = cv::imdecode(cv::Mat(msg.data), cv::IMREAD_COLOR).size();
    const double aspect_ratio = static_cast<double>(image_size.width) / image_size.height;

    // Set the size and position of the display window.
    const int window_width = static_cast<int>(monitor_geometry.width * window_width_ratio_);
    const int window_height = static_cast<int>(window_width / aspect_ratio);
    cv::resizeWindow(window_name_, window_width, window_height);

    const int window_x =
        monitor_geometry.x + static_cast<int>((monitor_geometry.width - window_width) * window_position_ratio_.x);
    const int window_y =
        monitor_geometry.y + static_cast<int>((monitor_geometry.height - window_height) * window_position_ratio_.y);
    cv::moveWindow(window_name_, window_x, window_y);
}

void CompressedImageViewer::printParam() const {
    RCLCPP_INFO(this->get_logger(), "[%s] ===============", __func__);
    RCLCPP_INFO(this->get_logger(), "  GDK_VERSION : %d\n", GDK_VERSION);

    RCLCPP_INFO(this->get_logger(), "(compressed_image_viewer.yaml)");
    RCLCPP_INFO(this->get_logger(), "  display_full_screen_   : %s", display_full_screen_ ? "true" : "false");

    if (!display_full_screen_) {
        RCLCPP_INFO(this->get_logger(), "  target_screen_idx_     : %d", target_screen_idx_);
        RCLCPP_INFO(this->get_logger(), "  window_width_ratio_    : %.2lf", window_width_ratio_);
        RCLCPP_INFO(this->get_logger(), "  window_position_ratio_ : (%.2lf, %.2lf)", window_position_ratio_.x,
                    window_position_ratio_.y);
    }
    RCLCPP_INFO(this->get_logger(), "============================\n");
}

void CompressedImageViewer::compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Compressed Image !");
    cv::imshow(window_name_, cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR));
    cv::waitKey(1);
}

}  // namespace aiformula
