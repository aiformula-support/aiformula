#include "lane_line_publisher/lane_pixel_finder.hpp"

namespace aiformula {

LanePixelFinder::LanePixelFinder(const int& min_area, const int& tolerance)
    : min_area_(min_area), tolerance_(tolerance) {}

void LanePixelFinder::findLanePixels(const cv::Mat& binary_mask, LaneLines& lane_lines) const {
    cv::Mat denoised;
    denoiseMask(binary_mask, denoised);
    searchMask(denoised, lane_lines);
}

cv::Mat LanePixelFinder::visualizeLanePixels(const cv::Mat& binary_mask, const LaneLines& lane_lines) const {
    static const uchar GRAY = 127;
    static const cv::Vec3b BLUE(255, 127, 0);
    static const cv::Vec3b GREEN(127, 255, 0);
    static const int MARKER_SIZE = 2;

    cv::Mat visualization;
    cv::cvtColor(binary_mask * GRAY, visualization, cv::COLOR_GRAY2BGR);

    for (const auto& pixel : lane_lines.left.pixels)
        cv::drawMarker(visualization, pixel, GREEN, cv::MARKER_SQUARE, MARKER_SIZE);
    for (const auto& pixel : lane_lines.right.pixels)
        cv::drawMarker(visualization, pixel, GREEN, cv::MARKER_SQUARE, MARKER_SIZE);
    for (const auto& pixel : lane_lines.center.pixels)
        cv::drawMarker(visualization, pixel, BLUE, cv::MARKER_SQUARE, MARKER_SIZE);
    return visualization;
}

void LanePixelFinder::denoiseMask(const cv::Mat& binary_mask, cv::Mat& denoised) const {
    cv::Mat labels, stats, centroids;
    static const int connectivity = 8;
    const int num_labels =
        cv::connectedComponentsWithStats(binary_mask, labels, stats, centroids, connectivity, CV_16U);

    denoised = cv::Mat::zeros(binary_mask.size(), CV_8U);
    for (int label = 1; label < num_labels; ++label)
        if (stats.at<int>(label, cv::CC_STAT_AREA) > min_area_) cv::bitwise_or(denoised, labels == label, denoised);
}

void LanePixelFinder::searchMask(const cv::Mat& binary_mask, LaneLines& lane_lines) const {
    const auto cols = binary_mask.cols;
    const auto rows = binary_mask.rows;

    // Initializations.
    const int top = 0;
    const int bottom = rows - 1;
    int left = 0;
    int right = cols - 1;
    int center = (left + right) / 2;

    for (int row = bottom; row >= top; --row) {
        const auto row_ptr = binary_mask.ptr<uchar>(row);
        if (row_ptr[center]) break;

        bool found_left = false;
        const auto left_bound = std::max(0, left - tolerance_);
        for (int col = center; col >= left_bound; --col) {
            if (row_ptr[col]) {
                left = col;
                found_left = true;
                break;
            }
        }

        bool found_right = false;
        const auto right_bound = std::min(cols - 1, right + tolerance_);
        for (int col = center; col <= right_bound; ++col) {
            if (row_ptr[col]) {
                right = col;
                found_right = true;
                break;
            }
        }

        center = (left + right) / 2;

        if (found_left) lane_lines.left.pixels.emplace_back(left, row);
        if (found_right) lane_lines.right.pixels.emplace_back(right, row);
        if (found_left && found_right) lane_lines.center.pixels.emplace_back(center, row);
    }
}

}  // namespace aiformula
