#ifndef GET_ROS_PARAMETER_HPP
#define GET_ROS_PARAMETER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

namespace aiformula {

/*
Since `as_xxx()` changes depending on the type of template arguments,
each implementation must be defined separately.
*/

// Declaration ==========================================================================================
template <typename T>
T getParameterAsType(rclcpp::Node* const node_ptr, const std::string& param_name);

// Implementation =======================================================================================
template <>
inline bool getParameterAsType<bool>(rclcpp::Node* const node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_bool();
}

template <>
inline int getParameterAsType<int>(rclcpp::Node* const node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_int();
}

template <>
inline double getParameterAsType<double>(rclcpp::Node* const node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_double();
}

template <>
inline std::string getParameterAsType<std::string>(rclcpp::Node* const node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_string();
}

template <>
inline std::vector<bool> getParameterAsType<std::vector<bool>>(rclcpp::Node* const node_ptr,
                                                               const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_bool_array();
}

template <>
inline std::vector<long int> getParameterAsType<std::vector<long int>>(rclcpp::Node* const node_ptr,
                                                                       const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_integer_array();
}

template <>
inline std::vector<double> getParameterAsType<std::vector<double>>(rclcpp::Node* const node_ptr,
                                                                   const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_double_array();
}

template <>
inline std::vector<std::string> getParameterAsType<std::vector<std::string>>(rclcpp::Node* const node_ptr,
                                                                             const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_string_array();
}

/**
 * @brief If the parameter named `param_name` has no value set, output error statement and shutdown ros.
 * If `param_name` type does not match `T`, output error statement and shutdown ros.
 *
 * @param[in] node_ptr node pointer
 * @param[in] param_name Rosparam Name
 * @return Value set for the parameter named `param_name`.
 *
 * @note Usage: `double ret = getRosParameter<double>(this, "test.value");`
 */
template <typename T>
T getRosParameter(rclcpp::Node* const node_ptr, const std::string& param_name) {
    if (!node_ptr->has_parameter(param_name)) {
        node_ptr->declare_parameter(param_name);
    }
    try {
        return getParameterAsType<T>(node_ptr, param_name);
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "Parameter '%s' is not declared", e.what());
        rclcpp::shutdown();
        exit(1);
    } catch (const rclcpp::ParameterTypeException& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "Parameter '%s' has an incorrect type: %s", param_name.c_str(), e.what());
        rclcpp::shutdown();
        exit(1);
    }
}

}  // namespace aiformula

#endif  // GET_ROS_PARAMETER_HPP
