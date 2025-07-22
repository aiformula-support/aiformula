import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from common_python.launch_util import get_frame_ids_and_topic_names

PACKAGE_NAME = "emergency_stop_controller"
PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)
_, TOPIC_NAMES = get_frame_ids_and_topic_names()


def generate_launch_description() -> LaunchDescription:
    launch_args = ()

    ROS_PARAM_CONFIG = (osp.join(PACKAGE_DIR, "config", PACKAGE_NAME + ".yaml"),)
    emergency_stop_controller = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_control",
        output="screen",
        emulate_tty=True,
        parameters=[*ROS_PARAM_CONFIG],
        remappings=[
            ("sub_can", TOPIC_NAMES["sensing"]["input_can_data"]),
            ("pub_twist", TOPIC_NAMES["control"]["speed_command"]["emergency_stop_controller"]),
        ],
    )

    return LaunchDescription(
        [
            *launch_args,
            emergency_stop_controller,
        ]
    )
