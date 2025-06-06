import os.path as osp
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "twist_mux"
    LAUNCHERS_DIR = get_package_share_directory("sample_launchers")
    FRAME_IDS, TOPIC_NAMES = get_frame_ids_and_topic_names()

    launch_args = (
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logger level (debug, info, warn, error, fatal)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="If true, run rviz2",
        ),
        DeclareLaunchArgument(
            "use_runtime_monitor",
            default_value="false",
            description="If true, run the GUI for diagnostics",
        ),
    )

    ROS_PARAM_CONFIG = (
        osp.join(LAUNCHERS_DIR, "config", "twist_mux.yaml"),
    )
    twist_mux = Node(
        package=PACKAGE_NAME,
        executable="twist_mux",
        name="twist_mux",
        namespace="/aiformula_control",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration('log_level')],
        parameters=[*ROS_PARAM_CONFIG,
                    {
                        "topics.handle_controller.topic": TOPIC_NAMES["control"]["speed_command"]["handle_controller"]["normal"],
                        "topics.gamepad.topic": TOPIC_NAMES["control"]["speed_command"]["gamepad"],
                        "topics.mpc.topic": TOPIC_NAMES["control"]["speed_command"]["mpc"],
                        "topics.handle_controller_coasting.topic": TOPIC_NAMES["control"]["speed_command"]["handle_controller"]["coasting"],
                        "locks.gamepad.topic": TOPIC_NAMES["control"]["twist_mux_lock"]["gamepad"],
                    }],
        remappings=[
            ("cmd_vel_out", TOPIC_NAMES["control"]["speed_command"]["multiplexed"]),
        ],
    )
    twist_marker = Node(
        package=PACKAGE_NAME,
        executable="twist_marker",
        name="twist_marker",
        namespace="/aiformula_visualization",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "frame_id": FRAME_IDS["base_footprint"],
            "scale": 1.0,
            "vertical_position": 2.0
        }],
        remappings=[
            ("twist", TOPIC_NAMES["control"]["speed_command"]["multiplexed"]),
        ],
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_" + PACKAGE_NAME,
        arguments=["-d", osp.join(LAUNCHERS_DIR, "rviz", "twist_marker.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    runtime_monitor = Node(
        package="rqt_runtime_monitor",
        executable="rqt_runtime_monitor",
        name="rqt_runtime_monitor_" + PACKAGE_NAME,
        condition=IfCondition(LaunchConfiguration("use_runtime_monitor")),
    )

    return LaunchDescription([
        *launch_args,
        twist_mux,
        twist_marker,
        rviz2,
        runtime_monitor,
    ])
