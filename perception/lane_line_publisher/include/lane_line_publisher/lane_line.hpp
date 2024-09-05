#ifndef LANE_LINE_HPP
#define LANE_LINE_HPP

#include <opencv2/opencv.hpp>

#include "common_cpp/tf2_transform.hpp"
#include "lane_line_publisher/cubic_line_fitter.hpp"
#include "lane_line_publisher/parametrized_polyline.hpp"

namespace aiformula {

class LaneLine {
public:
    explicit LaneLine() {}
    ~LaneLine() = default;

    void toVehicleFrame(const cv::Mat& camera_matrix, const tf2::Transform& vehicle_T_camera);
    void cropToRoi(const double& xmin, const double& xmax, const double& ymin, const double& ymax);
    void respacePoints(const double& spacing);
    void fitPoints(const CubicLineFitter::ConstPtr& fitter);

    std::vector<cv::Point2i> pixels;
    std::vector<Eigen::Vector3d> points;
};

enum { LEFT, RIGHT, CENTER, NUM_LANE_LINES };

class LaneLines {
public:
    explicit LaneLines() {}
    ~LaneLines() = default;

    LaneLine left;
    LaneLine right;
    LaneLine center;
};

}  // namespace aiformula

#endif  // LANE_LINE_HPP
