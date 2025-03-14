import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "image_compressor"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()

    launch_args = (
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logger level (debug, info, warn, error, fatal)",
        ),
        DeclareLaunchArgument(
            "use_rosbag",
            default_value="false",
            description="If true, play rosbag",
        ),
        DeclareLaunchArgument(
            "rosbag_path",
            default_value="~/data/aiformula/20240704_shiho_zed_can_vecnav/01_in_front_of_gate/01_in_front_of_gate_vehicle_info",
            description="Path of rosbag to play",
        ),
    )

    ROS_PARAM_CONFIG = (
        osp.join(PACKAGE_DIR, "config", PACKAGE_NAME + ".yaml"),
    )
    image_compressor = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_visualization",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[ROS_PARAM_CONFIG],
        remappings=[
            ("sub_image", TOPIC_NAMES["sensing"]["zedx"]["left_image"]["undistorted"]),
            ("pub_image", TOPIC_NAMES["visualization"]["aiformula_pilot"]),
        ],
    )
    rosbag_play = ExecuteProcess(
        cmd=[
            "ros2 bag play",
            " --topics ",
            TOPIC_NAMES["sensing"]["zedx"]["left_image"]["undistorted"],
            " -l ",
            LaunchConfiguration("rosbag_path"),
        ],
        condition=IfCondition(LaunchConfiguration("use_rosbag")),
        shell=True,
    )

    return LaunchDescription([
        *launch_args,
        image_compressor,
        rosbag_play,
    ])
