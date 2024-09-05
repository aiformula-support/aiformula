cmake_minimum_required(VERSION 3.5)
project(lane_line_publisher)
add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(common_cpp REQUIRED)

add_executable(lane_line_publisher
  src/cubic_line_fitter.cpp
  src/lane_line_publisher_node.cpp
  src/lane_line_publisher.cpp
  src/lane_line.cpp
  src/lane_pixel_finder.cpp
  src/parametrized_polyline.cpp
)
target_include_directories(lane_line_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(lane_line_publisher
  rclcpp
  cv_bridge
  PCL
  pcl_conversions
  sensor_msgs
  visualization_msgs
  Eigen3
  OpenCV
  common_cpp
)

install(TARGETS
  lane_line_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
