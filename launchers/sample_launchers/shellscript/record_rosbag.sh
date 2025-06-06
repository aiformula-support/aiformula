#!/bin/bash
<< COMMENTOUT
<Usage Example>
record_rosbag.sh
record_rosbag.sh 
COMMENTOUT

SCRIPT_DIR=$(cd $(dirname $0); pwd)
topic_list_yaml_path="${SCRIPT_DIR}/../config/topic_list.yaml"
read_yaml() {
    node_path=$1
    python3 -c "import yaml; print(yaml.safe_load(open('$topic_list_yaml_path'))${node_path})" 2>/dev/null
}

bag_name=${1:-test}

date_str=`date '+%Y%m%d'`
bag_dir="${HOME}/rosbag/${date_str}"
mkdir -p ${bag_dir}

ros2 bag record -o "${bag_dir}"/"${bag_name}" \
    --qos-profile-overrides-path ${SCRIPT_DIR}/../config/qos_setting.yaml \
    $(read_yaml "['sensing']['zedx']['left_image']['undistorted']") \
    $(read_yaml "['sensing']['zedx']['imu']") \
    $(read_yaml "['sensing']['input_can_data']") \
    $(read_yaml "['sensing']['odometry']['gyro']") \
    $(read_yaml "['sensing']['rear_potentiometer']") \
    /tf \
    /tf_static
