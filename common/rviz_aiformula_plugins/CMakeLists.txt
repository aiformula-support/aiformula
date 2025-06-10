cmake_minimum_required(VERSION 3.5)
project(rviz_aiformula_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(aiformula_interfaces REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
    include/rviz_aiformula_plugins/object_info_multi_array_displayer.hpp
)

# object_info_multi_array_displayer
add_library(object_info_multi_array_displayer
    src/object_info_multi_array_displayer.cpp
    ${MOC_FILES}
)
target_include_directories(object_info_multi_array_displayer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(object_info_multi_array_displayer
    pluginlib
    rviz_common
    rviz_rendering
    aiformula_interfaces
)

install(TARGETS
            object_info_multi_array_displayer
        EXPORT export_rviz_aiformula_plugins
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons/classes"
        DESTINATION share/${PROJECT_NAME}/icons
)
ament_export_include_directories(include)
ament_export_targets(export_rviz_aiformula_plugins)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
