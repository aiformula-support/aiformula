import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    PACKAGE_NAME = "image_compressor"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)

    launch_args = (
        # DeclareLaunchArgument(
        #     "logger",
        #     default_value="info",
        #     choices=["debug", "info", "warn", "error", "fatal"],
        #     description="Ros logger level",
        # ),
        DeclareLaunchArgument(
            "use_rosbag",
            default_value="false",
            description="If true, play rosbag",
        ),
        DeclareLaunchArgument(
            "rosbag_path",
            default_value="/workspace/data/20240704_shiho_zed_can_vecnav/01_in_front_of_gate/01_in_front_of_gate_vehicle_info",
            description="Path of rosbag to play",
        ),
    )

    ROS_PARAM_CONFIG = (osp.join(PACKAGE_DIR, "config", PACKAGE_NAME + ".yaml"),)
    image_compressor = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_visualization",
        output="screen",
        emulate_tty=True,
        # arguments=[
        #     "--ros-args",
        #     "--log-level",
        #     ["aiformula_visualization.", PACKAGE_NAME, ":=", LaunchConfiguration("logger")],
        # ],
        parameters=[ROS_PARAM_CONFIG],
        remappings=[
            ("sub_image", "/aiformula_sensing/zed_node/left_image/undistorted"),
            ("pub_image", "/aiformula_visualization/zed/left_image/compressed"),
        ],
    )
    rosbag_play = ExecuteProcess(
        cmd=[
            "ros2 bag play",
            " --topics ",
            "/aiformula_sensing/zed_node/left_image/undistorted",
            " -l ",
            LaunchConfiguration("rosbag_path"),
        ],
        condition=IfCondition(LaunchConfiguration("use_rosbag")),
        shell=True,
    )

    return LaunchDescription(
        [
            *launch_args,
            image_compressor,
            rosbag_play,
        ]
    )
