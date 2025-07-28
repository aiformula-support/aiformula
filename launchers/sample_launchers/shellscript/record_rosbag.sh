#!/bin/bash
<< COMMENTOUT
<Usage Example>
record_rosbag.sh
record_rosbag.sh zed_image_data
COMMENTOUT

SCRIPT_DIR=$(cd $(dirname $0); pwd)

bag_name=${1:-test}

date_str=`date '+%Y%m%d'`
bag_dir="${HOME}/rosbag/${date_str}"
mkdir -p ${bag_dir}

ros2 bag record -o "${bag_dir}"/"${bag_name}" \
    --qos-profile-overrides-path ${SCRIPT_DIR}/../../../common/aiformula_interfaces/config/qos_setting.yaml \
    /aiformula_sensing/zed_node/left_image/undistorted \
    /aiformula_sensing/zed_node/imu \
    /aiformula_sensing/vehicle_info \
    /aiformula_sensing/gyro_odometry_publisher/odom \
    /aiformula_sensing/rear_potentiometer/yaw \
    /tf \
    /tf_static
