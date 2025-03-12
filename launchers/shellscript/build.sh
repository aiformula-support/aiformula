#!/bin/bash
set -e

TARGET_PACKEGES=(
    launchers
    common_python
    common_cpp
    vehicle

    # sensing
    odometry_publisher
    rear_potentiometer
    zed_wrapper

    # perception
    lane_line_publisher

    # control
    motor_controller

    # remote control
    image_compressor

    # external repository
    vectornav
    ros2_socketcan
    twist_mux
)

echo "Build Targets:"
for packege in ${TARGET_PACKEGES[@]}; do
    echo " - ${packege}"
done
echo ""

cd ${HOME}/workspace/ros
colcon build --symlink-install --packages-up-to ${TARGET_PACKEGES[@]}
