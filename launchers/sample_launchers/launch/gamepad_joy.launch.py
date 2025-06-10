import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "joy"
    NODE_NAME = "joy_node"
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()

    ROS_PARAM_CONFIG = (
        osp.join(get_package_share_directory("sample_launchers"), "config", "gamepad_joy.yaml"),
    )
    gamepad_joy = Node(
        package=PACKAGE_NAME,
        executable=NODE_NAME,
        name=NODE_NAME,
        namespace="/aiformula_control/gamepad",
        parameters=[*ROS_PARAM_CONFIG],
        remappings=[
            ("joy", TOPIC_NAMES["control"]["joy"]["gamepad"]),
        ],
    )

    return LaunchDescription([
        gamepad_joy,
    ])
