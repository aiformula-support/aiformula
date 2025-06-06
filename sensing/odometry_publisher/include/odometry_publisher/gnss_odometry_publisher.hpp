#ifndef GNSS_ODOMETRY_PUBLISHER_HPP
#define GNSS_ODOMETRY_PUBLISHER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Original
#include "common_cpp/get_ros_parameter.hpp"
#include "odometry_publisher/odometry_publisher.hpp"

namespace aiformula {

class GnssOdometryPublisher : public OdometryPublisher {
public:
    explicit GnssOdometryPublisher();
    ~GnssOdometryPublisher() = default;

private:
    void getRosParams();
    void initValues();
    void printParam() const;
    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void latLonToPlaneRectXY(const double& phi, const double& lambda, double& x, double& y);
    void updatePosition(const double& x, const double& y, const double& dt);
    void updateYawAngleAndYawRate(const double& x, const double& y, const double& dt);

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;

    const double R2D = 180. / M_PI;
    const double D2R = M_PI / 180.;

    const double a_ = 6378137.;       // Long radius of ellipsoid (ITRF coordinate system GRS80 ellipsoid)
    const double F_ = 298.257222101;  // Inverse flatness of ellipsoid (ITRF coordinate system GRS80 ellipsoid)
    const double m0_ = 0.9999;        // Scale factor on the X-axis of the plane rectangular coordinate system
    const double n_ = 1. / (2. * F_ - 1.);

    std::array<double, 6> A_{};      // A0 ~ A5
    std::array<double, 6> alpha_{};  // α1 ~ α5

    // Latitude and longitude of the plane rectangular coordinate system origin
    // Use the value at the start of the program
    double phi0_;
    double lambda0_;

    // S_φ0, A_
    double S_phi0_;
    double A_bar_;
};

}  // namespace aiformula

#endif  // GNSS_ODOMETRY_PUBLISHER_HPP

/* Reference
https://gist.github.com/yubeneko/cbde0e3bae30eac4fab5457dc5b1f0a2
https://sw1227.hatenablog.com/entry/2018/11/30/200702
http://k-ichikawa.blog.enjoy.jp/etc/HP/js/sokuchi/sok2S.html
*/