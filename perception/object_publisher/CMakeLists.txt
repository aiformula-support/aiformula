cmake_minimum_required(VERSION 3.5)
project(object_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(common_cpp REQUIRED)
find_package(aiformula_interfaces REQUIRED)

file(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_executable(${PROJECT_NAME}
    ${SOURCE_FILES}
)
target_include_directories(${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_options(${PROJECT_NAME} PRIVATE
    -std=c++17
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    visualization_msgs
    OpenCV
    common_cpp
    aiformula_interfaces
)
install(TARGETS ${PROJECT_NAME}
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
