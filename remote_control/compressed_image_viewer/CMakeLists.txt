cmake_minimum_required(VERSION 3.5)
project(compressed_image_viewer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(common_cpp REQUIRED)

# Find the same version of the GDK library used by OpenCV
execute_process(
    COMMAND ${OpenCV_INSTALL_PATH}/bin/opencv_version --verbose
    OUTPUT_VARIABLE OPENCV_INFO
)
string(REGEX REPLACE "\n" ";" OPENCV_INFO "${OPENCV_INFO}")
set(GTK_VERSION "0")
foreach (line IN LISTS OPENCV_INFO)
    if (line MATCHES "GTK\\+:.*YES.*\\(ver ([0-9])")
        set(GTK_VERSION "${CMAKE_MATCH_1}")
        break()
    endif()
endforeach()
message(WARNING "GTK_VERSION: ${GTK_VERSION}")
if (GTK_VERSION STREQUAL "2")
    pkg_check_modules(GDK REQUIRED gdk-2.0)
elseif (GTK_VERSION STREQUAL "3")
    pkg_check_modules(GDK REQUIRED gdk-3.0)
else()
    message(FATAL_ERROR "Unsupported GTK version detected or not found!")
endif()


file(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_executable(${PROJECT_NAME}
    ${SOURCE_FILES}
)
target_include_directories(${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GDK_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    OpenCV
    common_cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    GDK_VERSION=${GTK_VERSION}
)
target_link_libraries(${PROJECT_NAME}
    ${GDK_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${CMAKE_CURRENT_SOURCE_DIR}/launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
