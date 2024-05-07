#include "common_cpp/tf2_transform.hpp"

namespace aiformula {

/**
 * @brief Get tf2 transformation from source to destination frame.
 *
 * @param[in] node_ptr node pointer
 * @param[in] dst_frame_id Destination frame ID.
 * @param[in] src_frame_id Source frame ID.
 * @return Tf2 transformation from source to destination frame.
 *
 * @note Usage: `auto vehicle_T_camera = getTf2Transform(this, "base_link", "zed_left_camera_optical_frame");`
 */
tf2::Transform getTf2Transform(rclcpp::Node* const node_ptr, const std::string& dst_frame_id,
                               const std::string& src_frame_id) {
    static tf2_ros::Buffer tf_buffer(node_ptr->get_clock());
    static tf2_ros::TransformListener tf_listener(tf_buffer);

    // wait a little for an instance of tf2_ros::TransformListener
    static bool is_first = true;
    if (is_first && !(is_first = false)) rclcpp::sleep_for(std::chrono::milliseconds(200));

    geometry_msgs::msg::TransformStamped stamped_tf_dst_T_src;
    try {
        stamped_tf_dst_T_src =
            tf_buffer.lookupTransform(dst_frame_id, src_frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "[%s] Could not transform '%s' to '%s': %s !", __func__,
                     dst_frame_id.c_str(), src_frame_id.c_str(), e.what());
        rclcpp::shutdown();
        exit(1);
    }
    const geometry_msgs::msg::Quaternion& rotation = stamped_tf_dst_T_src.transform.rotation;
    const geometry_msgs::msg::Vector3& translation = stamped_tf_dst_T_src.transform.translation;
    tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
    tf2::Vector3 t(translation.x, translation.y, translation.z);
    return tf2::Transform(q, t);
}  // getTf2Transform

/**
 * @brief Transform single image pixel to single three-dimensional point in the vehicle frame.
 *
 * @param[in] pixel Image pixel to be transformed.
 * @param[in] invert_camera_matrix Inverse of camera matrix.
 * @param[in] vehicle_T_camera dst_frame_id: vehicle,  src_frame_id: camera.
 * @param[out] vehicle_point Transformed three-dimensional point in the vehicle frame.
 * @return True: Transform succeeded and gave the value to 'vehicle_point'.
 *         False: Transform failed and didn't give the value to 'vehicle_point'
 *
 * @note Usage: `if (pixelToPoint(pixel, invert_camera_matrix, vehicle_T_camera, vehicle_point))`
 */
bool pixelToPoint(const cv::Point2f& pixel, const cv::Mat& invert_camera_matrix, const tf2::Transform& vehicle_T_camera,
                  tf2::Vector3& vehicle_point) {
    // Transform from pixel to camera vector
    const cv::Mat pixel_matrix = (cv::Mat_<float>(3, 1) << pixel.x, pixel.y, 1.0);
    const cv::Mat camera_vec_matrix = invert_camera_matrix * pixel_matrix;

    // Transform from camera vector to vehicle point
    const tf2::Vector3 camera_vec(camera_vec_matrix.at<float>(0, 0), camera_vec_matrix.at<float>(1, 0),
                                  camera_vec_matrix.at<float>(2, 0));
    const auto vehicle_vec = tf2::quatRotate(vehicle_T_camera.getRotation(), camera_vec);
    const auto scale = -vehicle_T_camera.getOrigin().z() / vehicle_vec.z();
    if (scale < 0.) return false;
    vehicle_point = vehicle_T_camera * (scale * camera_vec);
    return true;
}  // pixelToPoint

/**
 * @brief Transform multi image pixels to multi three-dimensional points in the vehicle frame.
 * Skip pixels that failed to be transformed.
 *
 * @param[in] pixels Image pixels to be transformed.
 * @param[in] invert_camera_matrix Inverse of camera matrix.
 * @param[in] vehicle_T_camera dst_frame_id: vehicle,  src_frame_id: camera.
 * @return Transformed three-dimensional points in the vehicle frame.
 *
 * @note Usage: `auto vehicle_points = pixelsToPoints(pixels, invert_camera_matrix, vehicle_T_camera)`
 */
std::vector<tf2::Vector3> pixelsToPoints(const std::vector<cv::Point2f>& pixels, const cv::Mat& invert_camera_matrix,
                                         const tf2::Transform& vehicle_T_camera) {
    std::vector<tf2::Vector3> vehicle_points;
    for (const auto pixel : pixels) {
        tf2::Vector3 vehicle_point;
        if (pixelToPoint(pixel, invert_camera_matrix, vehicle_T_camera, vehicle_point)) {
            vehicle_points.emplace_back(vehicle_point);
        }
    }
    return vehicle_points;
}  // pixelsToPoints

/**
 * @brief Transform single three-dimensional point in the vehicle frame to single image pixel.
 *
 * @param[in] vehicle_point Three-dimensional point to be transformed to vehicle frame.
 * @param[in] camera_matrix Camera matrix.
 * @param[in] vehicle_T_camera dst_frame_id: vehicle,  src_frame_id: camera.
 * @param[out] pixel Transformed image pixel
 * @param[in] valid_roi Region within which the transformed pixel is considered valid.
 * @return True  : Transformed pixel is inside the 'valid_roi' or 'valid_roi' isn't given
           False : Transformed pixel is outside the 'valid_roi'
 *
 * @note Usage: `if (pointToPixel(vehicle_point, camera_matrix, camera_T_vehicle, pixel))`
 *              `if (pointToPixel(vehicle_point_outside_image, camera_matrix, camera_T_vehicle, ret_pixel, &valid_roi))`
 */
bool pointToPixel(const tf2::Vector3& vehicle_point, const cv::Mat& camera_matrix,
                  const tf2::Transform& camera_T_vehicle, cv::Point2f& pixel, const cv::Rect* valid_roi) {
    // Transform from vehicle point to camera vector
    const auto camera_point = camera_T_vehicle * vehicle_point;
    const cv::Mat camera_vec_matrix =
        (cv::Mat_<float>(3, 1) << camera_point.x() / camera_point.z(), camera_point.y() / camera_point.z(), 1.0);

    // Transform  from camera vector ot pixel
    const cv::Mat pixel_matrix = camera_matrix * camera_vec_matrix;
    pixel = cv::Point2f(pixel_matrix.at<float>(0, 0), pixel_matrix.at<float>(1, 0));
    if (valid_roi) {
        return valid_roi->contains(pixel);
    } else {
        return true;
    }
}  // pointToPixel

/**
 * @brief Transform multi three-dimensional point in the vehicle frame to multi image pixel.
 * if 'valid_roi' is given, skip vehicle_points if transformed pixel is outside the 'valid_roi'.
 * if 'valid_roi' isn't given, all vehicle_points are transformed.
 *
 * @param[in] vehicle_points Three-dimensional points to be transformed to vehicle frame.
 * @param[in] camera_matrix Camera matrix.
 * @param[in] vehicle_T_camera dst_frame_id: vehicle,  src_frame_id: camera.
 * @param[in] valid_roi Region within which the transformed pixel is considered valid.
 * @return Transformed image pixels
 *
 * @note Usage: `auto pixels = pointsToPixels(vehicle_points, camera_matrix, camera_T_vehicle);`
 *              `auto pixels
 *                   = pointsToPixels(vehicle_points_outside_image, camera_matrix, camera_T_vehicle, &valid_roi);`
 */
std::vector<cv::Point2f> pointsToPixels(const std::vector<tf2::Vector3>& vehicle_points, const cv::Mat& camera_matrix,
                                        const tf2::Transform& camera_T_vehicle, const cv::Rect* valid_roi) {
    std::vector<cv::Point2f> pixels;
    for (const auto vehicle_point : vehicle_points) {
        cv::Point2f pixel;
        if (pointToPixel(vehicle_point, camera_matrix, camera_T_vehicle, pixel, valid_roi)) {
            pixels.emplace_back(pixel);
        }
    }
    return pixels;
}  // pointsToPixels

}  // namespace aiformula
