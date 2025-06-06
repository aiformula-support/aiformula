cmake_minimum_required(VERSION 3.5)
project(odometry_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(common_cpp REQUIRED)

# wheel_odometry_publisher
add_executable(wheel_odometry_publisher
    ${CMAKE_CURRENT_SOURCE_DIR}/src/odometry_publisher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/wheel_odometry_publisher/wheel_odometry_publisher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/wheel_odometry_publisher/wheel_odometry_publisher_node.cpp
)
target_include_directories(wheel_odometry_publisher PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(wheel_odometry_publisher
    rclcpp
    tf2
    tf2_ros
    can_msgs
    nav_msgs
    common_cpp
)
install(TARGETS wheel_odometry_publisher
    DESTINATION lib/${PROJECT_NAME}
)

# gyro_odometry_publisher
add_executable(gyro_odometry_publisher
    ${CMAKE_CURRENT_SOURCE_DIR}/src/odometry_publisher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gyro_odometry_publisher/gyro_odometry_publisher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gyro_odometry_publisher/gyro_odometry_publisher_node.cpp
)
target_include_directories(gyro_odometry_publisher PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(gyro_odometry_publisher
    rclcpp
    tf2_ros
    sensor_msgs
    can_msgs
    nav_msgs
    common_cpp
)
install(TARGETS gyro_odometry_publisher
    DESTINATION lib/${PROJECT_NAME}
)

# gnss_odometry_publisher
add_executable(gnss_odometry_publisher
    ${CMAKE_CURRENT_SOURCE_DIR}/src/odometry_publisher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gnss_odometry_publisher/gnss_odometry_publisher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gnss_odometry_publisher/gnss_odometry_publisher_node.cpp
)
target_include_directories(gnss_odometry_publisher PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(gnss_odometry_publisher
    rclcpp
    tf2_ros
    sensor_msgs
    nav_msgs
    common_cpp
)
install(TARGETS gnss_odometry_publisher
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${CMAKE_CURRENT_SOURCE_DIR}/launch
        ${CMAKE_CURRENT_SOURCE_DIR}/rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
