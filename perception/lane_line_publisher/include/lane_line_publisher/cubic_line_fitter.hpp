#ifndef CUBIC_LINE_FITTER_HPP
#define CUBIC_LINE_FITTER_HPP

#include <eigen3/Eigen/Geometry>
#include <memory>

namespace aiformula {

class CubicLineFitter {
public:
    explicit CubicLineFitter(const double& xmin, const double& xmax, const double& spacing);
    ~CubicLineFitter() = default;

    using ConstPtr = std::shared_ptr<const CubicLineFitter>;

    void fitAndExtrapolate(const std::vector<Eigen::Vector3d>& points,
                           std::vector<Eigen::Vector3d>& extrapolated) const;

private:
    void calcCoefficients(const std::vector<Eigen::Vector3d>& points, Eigen::Vector4d& coefficients) const;

    int rank_, num_points_;
    Eigen::ArrayXd x_coords_;
    Eigen::MatrixXd x_powers_;
};

}  // namespace aiformula

#endif  // CUBIC_LINE_FITTER_HPP
