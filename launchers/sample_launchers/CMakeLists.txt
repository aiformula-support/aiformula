cmake_minimum_required(VERSION 3.5)
project(sample_launchers)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${CMAKE_CURRENT_SOURCE_DIR}/launch
        ${CMAKE_CURRENT_SOURCE_DIR}/rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
