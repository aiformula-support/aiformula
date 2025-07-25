import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    PACKAGE_NAME = "joy"
    NODE_NAME = "joy_node"

    ROS_PARAM_CONFIG = (osp.join(get_package_share_directory("sample_launchers"), "config", "gamepad_joy.yaml"),)
    gamepad_joy = Node(
        package=PACKAGE_NAME,
        executable=NODE_NAME,
        name=NODE_NAME,
        namespace="/aiformula_control/gamepad",
        parameters=[*ROS_PARAM_CONFIG],
        remappings=[
            ("joy", "/aiformula_control/gamepad/joy"),
        ],
    )

    return LaunchDescription(
        [
            gamepad_joy,
        ]
    )
