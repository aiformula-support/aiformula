#!/bin/bash
set -e

TARGET_PACKEGES=(
    launchers
    common_python
    common_cpp
    vehicle

    # sensing
    odometry_publisher
    zed_wrapper

    # control
    motor_controller

    # external repository
    vectornav
    ros2_socketcan
)

echo "Build Targets:"
for packege in ${TARGET_PACKEGES[@]}; do
    echo " - ${packege}"
done
echo ""

cd ${HOME}/workspace/ros
colcon build --symlink-install --packages-up-to ${TARGET_PACKEGES[@]}
