#include "odometry_publisher/gnss_odometry_publisher.hpp"
namespace aiformula {

GnssOdometryPublisher::GnssOdometryPublisher()
    : OdometryPublisher("gnss_odometry_publisher"), phi0_(0.0), lambda0_(0.0), S_phi0_(0.0) {
    initValues();
}

void GnssOdometryPublisher::initValues() {
    const int buffer_size = 10;
    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "sub_gnss", buffer_size, std::bind(&GnssOdometryPublisher::gnssCallback, this, std::placeholders::_1));

    // The calculation methods for 'A_' and 'alpha_' can be found at the link at the bottom of
    // gnss_odometry_publisher.hpp.
    A_[0] = 1. + pow(n_, 2) / 4. + pow(n_, 4) / 64.;
    A_[1] = -3. / 2. * (n_ - pow(n_, 3) / 8. - pow(n_, 5) / 64.);
    A_[2] = 15. / 16. * (pow(n_, 2) - pow(n_, 4) / 4.);
    A_[3] = -35. / 48. * (pow(n_, 3) - 5. / 16. * pow(n_, 5));
    A_[4] = 315. / 512. * pow(n_, 4);
    A_[5] = -693. / 1280. * pow(n_, 5);

    alpha_[1] = 1. / 2. * n_ - 2. / 3. * pow(n_, 2) + 5. / 16. * pow(n_, 3) + 41. / 180. * pow(n_, 4) -
                127. / 288. * pow(n_, 5);
    alpha_[2] = 13. / 48. * pow(n_, 2) - 3. / 5. * pow(n_, 3) + 557. / 1440. * pow(n_, 4) + 281. / 630. * pow(n_, 5);
    alpha_[3] = 61. / 240. * pow(n_, 3) - 103. / 140. * pow(n_, 4) + 15061. / 26880. * pow(n_, 5);
    alpha_[4] = 49561. / 161280. * pow(n_, 4) - 179. / 168. * pow(n_, 5);
    alpha_[5] = 34729. / 80640. * pow(n_, 5);
}

void GnssOdometryPublisher::gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Gnss !");

    double x, y;
    latLonToPlaneRectXY(msg->latitude * D2R, msg->longitude * D2R, x, y);
    const double dt = calcTimeDelta(msg->header.stamp);
    updatePosition(x, y, dt);
    updateYawAngleAndYawRate(x, y, dt);
    nav_msgs::msg::Odometry odometry = createOdometryMsg(msg->header.stamp);
    odometry_pub_->publish(odometry);
    broadcastTf(odometry);
    RCLCPP_INFO_ONCE(this->get_logger(), "Publish Odometry !");
}

void GnssOdometryPublisher::latLonToPlaneRectXY(const double& phi, const double& lambda, double& x, double& y) {
    // At the first time, initialize phi0_, lambda0_, S_phi0_, A_bar_
    if (!phi0_ && !lambda0_) {
        phi0_ = phi;
        lambda0_ = lambda;

        for (int j = 1; j <= 5; j++) {
            S_phi0_ += A_[j] * sin(2 * j * phi0_);
        }
        S_phi0_ = m0_ * a_ / (1. + n_) * (A_[0] * phi0_ + S_phi0_);
        A_bar_ = m0_ * a_ / (1. + n_) * A_[0];
    }

    // λc, λs
    const double lambda_c = cos(lambda - lambda0_);
    const double lambda_s = sin(lambda - lambda0_);

    // t, t_
    const double t = sinh(atanh(sin(phi)) - 2. * sqrt(n_) / (1. + n_) * atanh(2. * sqrt(n_) / (1. + n_) * sin(phi)));
    const double t_bar = sqrt(1. + pow(t, 2));

    // ξ', η'
    const double xi_prime = atan(t / lambda_c);
    const double eta_prime = atanh(lambda_s / t_bar);

    // x, y
    x = y = 0.;
    for (int j = 1; j <= 5; j++) {
        x += alpha_[j] * sin(2. * j * xi_prime) * cosh(2. * j * eta_prime);
        y += alpha_[j] * cos(2. * j * xi_prime) * sinh(2. * j * eta_prime);
    }
    x = A_bar_ * (xi_prime + x) - S_phi0_;  // x-axis positive: North
    y = A_bar_ * (eta_prime + y);           // y-axis positive: East
    y *= -1.;                               // y-axis positive: East  -> West

    // true north angle
    double sigma = 1.0, tau = 0.;
    for (int j = 1; j <= 5; j++) {
        sigma += 2. * alpha_[j] * sin(2. * j * xi_prime) * cosh(2. * j * eta_prime);
        tau += 2. * alpha_[j] * cos(2. * j * xi_prime) * sinh(2. * j * eta_prime);
    }
    double gamma_prime =
        -atan2((tau * t_bar * lambda_c + sigma * t * lambda_s), (sigma * t_bar, lambda_c - tau * t * lambda_s));
    // std::cout << "gamma_prime: " << gamma_prime * R2D << std::endl;
}

void GnssOdometryPublisher::updatePosition(const double& x, const double& y, const double& dt) {
    vehicle_linear_velocity_.x = (x - pos_.x) / dt;
    vehicle_linear_velocity_.y = (y - pos_.y) / dt;
    pos_.x = x;
    pos_.y = y;
}

void GnssOdometryPublisher::updateYawAngleAndYawRate(const double& x, const double& y, const double& dt) {
    static double x_prev = DBL_MAX, y_prev = DBL_MAX;
    if (x_prev != DBL_MAX && y_prev != DBL_MAX) {
        double yaw = atan2(y - y_prev, x - x_prev);
        yaw_rate_ = (yaw - yaw_angle_) / dt;
        yaw_angle_ = yaw;
    }
    x_prev = x;
    y_prev = y;
}

}  // namespace aiformula
