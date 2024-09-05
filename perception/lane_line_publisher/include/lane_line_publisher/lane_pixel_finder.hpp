#ifndef LANE_PIXEL_FINDER_HPP
#define LANE_PIXEL_FINDER_HPP

#include "lane_line_publisher/lane_line.hpp"

namespace aiformula {

class LanePixelFinder {
public:
    explicit LanePixelFinder(const int& min_area, const int& tolerance);
    ~LanePixelFinder() = default;

    using ConstPtr = std::shared_ptr<const LanePixelFinder>;

    void findLanePixels(const cv::Mat& binary_mask, LaneLines& lane_lines) const;
    cv::Mat visualizeLanePixels(const cv::Mat& binary_mask, const LaneLines& lane_lines) const;

private:
    void denoiseMask(const cv::Mat& binary_mask, cv::Mat& denoised) const;
    void searchMask(const cv::Mat& binary_mask, LaneLines& lane_lines) const;

    int min_area_;   // [pix^2]
    int tolerance_;  // [pix]
};

}  // namespace aiformula

#endif  // LANE_PIXEL_FINDER_HPP
