#ifndef GET_ROS_PARAMETER_HPP
#define GET_ROS_PARAMETER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// Original
#include "common_cpp/util.hpp"

namespace aiformula {

class ParameterNotProvidedException : public std::runtime_error {
public:
    ParameterNotProvidedException(const std::string& message) : std::runtime_error(message) {}
};

/*
Since `as_xxx()` changes depending on the type of template arguments,
each implementation must be defined separately.
*/

// Declaration ==========================================================================================
template <typename T>
T getParameterAsType(const rclcpp::Parameter param);

// Implementation =======================================================================================
template <>
inline bool getParameterAsType<bool>(const rclcpp::Parameter param) {
    return param.as_bool();
}

template <>
inline int getParameterAsType<int>(const rclcpp::Parameter param) {
    return param.as_int();
}

template <>
inline double getParameterAsType<double>(const rclcpp::Parameter param) {
    return param.as_double();
}

template <>
inline std::string getParameterAsType<std::string>(const rclcpp::Parameter param) {
    return param.as_string();
}

template <>
inline std::vector<uint8_t> getParameterAsType<std::vector<uint8_t>>(const rclcpp::Parameter param) {
    return param.as_byte_array();
}

template <>
inline std::vector<bool> getParameterAsType<std::vector<bool>>(const rclcpp::Parameter param) {
    return param.as_bool_array();
}

template <>
inline std::vector<long int> getParameterAsType<std::vector<long int>>(const rclcpp::Parameter param) {
    return param.as_integer_array();
}

template <>
inline std::vector<double> getParameterAsType<std::vector<double>>(const rclcpp::Parameter param) {
    return param.as_double_array();
}

template <>
inline std::vector<std::string> getParameterAsType<std::vector<std::string>>(const rclcpp::Parameter param) {
    return param.as_string_array();
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
    try {
        node_ptr->declare_parameter(param_name, T());
        const auto param_overrides = node_ptr->get_node_parameters_interface()->get_parameter_overrides();
        if (param_overrides.find(param_name) == param_overrides.end()) {
            throw ParameterNotProvidedException("The value for '" + param_name + "' has not been provided");
        }
        return getParameterAsType<T>(node_ptr->get_parameter(param_name));
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "%s: %s !", toExceptionTypeString(e).c_str(), e.what());
        rclcpp::shutdown();
        exit(1);
    }
}

}  // namespace aiformula

#endif  // GET_ROS_PARAMETER_HPP
