import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "vectornav"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()

    ROS_PARAM_CONFIG = osp.join(PACKAGE_DIR, "config", "vectornav.yaml")
    vectornav = Node(
        package=PACKAGE_NAME,
        executable="vectornav",
        name="vectornav",
        namespace="/aiformula_sensing",
        parameters=[ROS_PARAM_CONFIG],
    )
    vn_sensor_msgs = Node(
        package=PACKAGE_NAME,
        executable="vn_sensor_msgs",
        name="vn_sensor_msgs",
        namespace="/aiformula_sensing",
        parameters=[ROS_PARAM_CONFIG],
        remappings=[
            ("vectornav/imu", TOPIC_NAMES["sensing"]["vectornav"]["imu"]),
        ]
    )

    return LaunchDescription([
        vectornav,
        vn_sensor_msgs,
    ])
