import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "teleop_twist_joy"
    NODE_NAME = "teleop_node"
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()

    launch_args = (
        DeclareLaunchArgument(
            "game_pad",
            default_value="dualshock4",
            description="GamePad type.",
        ),
        DeclareLaunchArgument(
            "button_layout_config",
            default_value=[
                osp.join(get_package_share_directory("launchers"),
                         "config/gamepad", ""), LaunchConfiguration("game_pad"), ".yaml"
            ],
            description="GamePad button layout configuration file.",
        ),
    )

    teleop_node = Node(
        package=PACKAGE_NAME,
        executable=NODE_NAME,
        name=NODE_NAME,
        namespace="/aiformula_control",
        parameters=[LaunchConfiguration("button_layout_config")],
        remappings=[
            ("joy", TOPIC_NAMES["control"]["game_pad"]),
            ("cmd_vel", TOPIC_NAMES["control"]["speed_command"]["game_pad"]),
        ],
    )

    return LaunchDescription([
        *launch_args,
        teleop_node,
    ])