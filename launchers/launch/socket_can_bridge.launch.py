import os.path as osp
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_DIR = get_package_share_directory("ros2_socketcan")
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()

    launch_args = (
        DeclareLaunchArgument(
            "interface", default_value="can0"),
        DeclareLaunchArgument(
            "receiver_interval_sec", default_value="0.01"),
        DeclareLaunchArgument(
            "sender_timeout_sec", default_value="0.01"),
        DeclareLaunchArgument(
            "enable_can_fd", default_value="false"),
    )

    can_receiver = GroupAction(
        actions=[
            PushRosNamespace("/aiformula_sensing"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    osp.join(PACKAGE_DIR, "launch/socket_can_receiver.launch.py"),
                ),
                launch_arguments={
                    "interface": LaunchConfiguration("interface"),
                    "interval_sec": LaunchConfiguration("receiver_interval_sec"),
                    "enable_can_fd": LaunchConfiguration("enable_can_fd"),
                    "from_can_bus_topic": TOPIC_NAMES["sensing"]["input_can_data"],
                }.items(),
            ),
        ]
    )

    can_sender = GroupAction(
        actions=[
            PushRosNamespace("/aiformula_control"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    osp.join(PACKAGE_DIR, "launch/socket_can_sender.launch.py"),
                ),
                launch_arguments={
                    "interface": LaunchConfiguration("interface"),
                    "interval_sec": LaunchConfiguration("sender_timeout_sec"),
                    "enable_can_fd": LaunchConfiguration("enable_can_fd"),
                    "to_can_bus_topic": TOPIC_NAMES["control"]["output_can_data"],
                }.items(),
            ),
        ]
    )

    return LaunchDescription([
        *launch_args,
        can_receiver,
        can_sender,
    ])
