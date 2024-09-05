#ifndef PARAMETRIZED_POLYLINE_HPP
#define PARAMETRIZED_POLYLINE_HPP

#include <eigen3/Eigen/Geometry>

namespace aiformula {

class ParametrizedPolyline {
public:
    explicit ParametrizedPolyline(const std::vector<Eigen::Vector3d>& points);
    ~ParametrizedPolyline() = default;

    const std::vector<Eigen::Vector3d>& points() const { return points_; }
    const double& length() const { return cumulative_lengths_.back(); }

    Eigen::Vector3d pointAt(const double& length) const;
    void pointsAt(const std::vector<double>& lengths, std::vector<Eigen::Vector3d>& points) const;

private:
    std::vector<Eigen::Vector3d> points_;
    std::vector<Eigen::Vector3d> normalized_segments_;
    std::vector<double> cumulative_lengths_;
};

}  // namespace aiformula

#endif  // PARAMETRIZED_POLYLINE_HPP
