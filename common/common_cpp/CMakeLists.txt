cmake_minimum_required(VERSION 3.5)
project(common_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(LIBRARY_NAME ${PROJECT_NAME})
set(DEPENDENCIES
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    geometry_msgs
    OpenCV
)
set(INCLUDE_DIRECTORY_NAME_AT_INSTALL "include")

file(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library(${LIBRARY_NAME} SHARED
    ${SOURCE_FILES}
)
target_include_directories(${LIBRARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${INCLUDE_DIRECTORY_NAME_AT_INSTALL}>
)
ament_target_dependencies(${LIBRARY_NAME}
    ${DEPENDENCIES}
)
install(TARGETS ${LIBRARY_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${INCLUDE_DIRECTORY_NAME_AT_INSTALL}
)

ament_export_include_directories(${INCLUDE_DIRECTORY_NAME_AT_INSTALL})
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${DEPENDENCIES})

ament_package()
