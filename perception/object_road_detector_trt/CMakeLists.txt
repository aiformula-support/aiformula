cmake_minimum_required(VERSION 3.18)
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 87)
endif()
project(object_road_detector_trt LANGUAGES CXX CUDA)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(aiformula_msgs REQUIRED)
find_package(common_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDAToolkit REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    HINTS /usr/include/aarch64-linux-gnu /usr/include
    REQUIRED)
find_library(NVINFER_LIB nvinfer REQUIRED)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/object_road_detector_trt.cpp
    src/trt_engine.cpp
    src/kernels.cu
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${TENSORRT_INCLUDE_DIR}
    ${CUDAToolkit_INCLUDE_DIRS}
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wpedantic>
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    CUDA_SEPARABLE_COMPILATION OFF
    POSITION_INDEPENDENT_CODE ON
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_components
    sensor_msgs
    aiformula_msgs
    common_cpp
    OpenCV
)
target_link_libraries(${PROJECT_NAME}
    CUDA::cudart
    ${NVINFER_LIB}
    ${NVINFER_PLUGIN_LIB}
)

rclcpp_components_register_nodes(${PROJECT_NAME} "aiformula::ObjectRoadDetectorTrt")

add_executable(${PROJECT_NAME}_node
    src/object_road_detector_trt_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
    DESTINATION include
)
install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
