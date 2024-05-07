cmake_minimum_required(VERSION 3.5)
project(launchers)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${CMAKE_CURRENT_SOURCE_DIR}/launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
