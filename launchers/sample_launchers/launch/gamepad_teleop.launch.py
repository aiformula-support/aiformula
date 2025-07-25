import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    PACKAGE_NAME = "teleop_twist_joy"
    NODE_NAME = "teleop_node"

    launch_args = (
        DeclareLaunchArgument(
            "gamepad",
            default_value="dualshock4",
            description="GamePad type.",
        ),
        DeclareLaunchArgument(
            "button_layout_config",
            default_value=[
                osp.join(get_package_share_directory("sample_launchers"), "config/gamepad", ""),
                LaunchConfiguration("gamepad"),
                ".yaml",
            ],
            description="GamePad button layout configuration file.",
        ),
    )

    gamepad_teleop = Node(
        package=PACKAGE_NAME,
        executable=NODE_NAME,
        name=NODE_NAME,
        namespace="/aiformula_control/gamepad",
        parameters=[LaunchConfiguration("button_layout_config")],
        remappings=[
            ("joy", "/aiformula_control/gamepad/joy"),
            ("cmd_vel", "/aiformula_control/gamepad/cmd_vel"),
        ],
    )

    return LaunchDescription(
        [
            *launch_args,
            gamepad_teleop,
        ]
    )
