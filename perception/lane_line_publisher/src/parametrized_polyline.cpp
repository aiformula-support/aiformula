#include "lane_line_publisher/parametrized_polyline.hpp"

namespace aiformula {

ParametrizedPolyline::ParametrizedPolyline(const std::vector<Eigen::Vector3d>& points)
    : points_(points), cumulative_lengths_{0.0} {
    const auto num_points = static_cast<int>(points.size());
    for (int i = 0; i < num_points - 1; ++i) {
        const auto segment = points[i + 1] - points[i];
        normalized_segments_.emplace_back(segment.normalized());
        cumulative_lengths_.emplace_back(cumulative_lengths_.back() + segment.norm());
    }
}

Eigen::Vector3d ParametrizedPolyline::pointAt(const double& length) const {
    // Handle extrapolation cases first.
    if (length < 0.0)
        return points_.front() + length * normalized_segments_.front();
    else if (length > this->length())
        return points_.back() + (length - this->length()) * normalized_segments_.back();

    // Locate the segment on which the interpolated point should lie, then interpolate.
    const auto right_it =
        std::lower_bound(cumulative_lengths_.begin(), cumulative_lengths_.end(), length, std::less_equal{});
    const auto left_it = std::prev(right_it);
    const auto index = std::distance(cumulative_lengths_.begin(), left_it);
    return points_[index] + (length - cumulative_lengths_[index]) * normalized_segments_[index];
}

void ParametrizedPolyline::pointsAt(const std::vector<double>& lengths, std::vector<Eigen::Vector3d>& points) const {
    std::transform(lengths.begin(), lengths.end(), std::back_inserter(points),
                   [this](const auto& length) { return this->pointAt(length); });
}

}  // namespace aiformula
