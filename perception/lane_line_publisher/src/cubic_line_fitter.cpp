#include "lane_line_publisher/cubic_line_fitter.hpp"

namespace aiformula {

CubicLineFitter::CubicLineFitter(const double& xmin, const double& xmax, const double& spacing) : rank_(4) {
    num_points_ = static_cast<int>((xmax - xmin) / spacing) + 1;
    x_coords_ = Eigen::ArrayXd(num_points_);
    for (int i = 0; i < num_points_; ++i) x_coords_(i) = xmin + spacing * i;

    x_powers_ = Eigen::MatrixXd(num_points_, rank_);
    x_powers_.col(0) = Eigen::VectorXd::Ones(num_points_);
    for (int i = 1; i < rank_; ++i) x_powers_.col(i) = x_coords_.pow(i);
}

void CubicLineFitter::fitAndExtrapolate(const std::vector<Eigen::Vector3d>& points,
                                        std::vector<Eigen::Vector3d>& extrapolated) const {
    Eigen::Vector4d coefficients;
    calcCoefficients(points, coefficients);

    const auto y_coords = x_powers_ * coefficients;
    for (int i = 0; i < num_points_; ++i) extrapolated.emplace_back(x_coords_(i), y_coords(i), 0.);
}

void CubicLineFitter::calcCoefficients(const std::vector<Eigen::Vector3d>& points,
                                       Eigen::Vector4d& coefficients) const {
    const int num_points = static_cast<int>(points.size());
    Eigen::ArrayXd x_coords(num_points);
    Eigen::VectorXd y_coords(num_points);
    for (int i = 0; i < num_points; ++i) {
        x_coords(i) = points[i].x();
        y_coords(i) = points[i].y();
    }

    Eigen::MatrixXd x_powers(num_points, rank_);
    x_powers.col(0) = Eigen::VectorXd::Ones(num_points);
    for (int i = 1; i < rank_; ++i) {
        x_powers.col(i) = x_coords.pow(i);
    }
    coefficients = x_powers.colPivHouseholderQr().solve(y_coords);
}

}  // namespace aiformula
