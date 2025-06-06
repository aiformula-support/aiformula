cmake_minimum_required(VERSION 3.5)
project(aiformula_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB MESSAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
rosidl_generate_interfaces(${PROJECT_NAME}
    ${MESSAGE_FILES}
    DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
