import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    PACKAGE_NAME = "twist_mux"
    LAUNCHERS_DIR = get_package_share_directory("sample_launchers")

    launch_args = (
        # DeclareLaunchArgument(
        #     "logger",
        #     default_value="info",
        #     choices=["debug", "info", "warn", "error", "fatal"],
        #     description="Ros logger level",
        # ),
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

    ROS_PARAM_CONFIG = (osp.join(LAUNCHERS_DIR, "config", "twist_mux.yaml"),)
    twist_mux = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_control",
        output="screen",
        emulate_tty=True,
        # arguments=[
        #     "--ros-args",
        #     "--log-level",
        #     ["aiformula_control.", PACKAGE_NAME, ":=", LaunchConfiguration("logger")],
        # ],
        parameters=[
            *ROS_PARAM_CONFIG,
            {
                "topics.emergency_stop_controller.topic": "/aiformula_control/emergency_stop_controller/cmd_vel",
                "topics.handle_controller.topic": "/aiformula_control/handle_controller/cmd_vel",
                "topics.gamepad.topic": "/aiformula_control/gamepad/cmd_vel",
                "topics.mpc.topic": "/aiformula_control/extremum_seeking_mpc/cmd_vel",
                "topics.handle_controller_coasting.topic": "/aiformula_control/handle_controller/cmd_vel/coasting",
                "locks.gamepad.topic": "/aiformula_control/twist_mux/gamepad/lock",
            },
        ],
        remappings=[
            ("cmd_vel_out", "/aiformula_control/twist_mux/cmd_vel"),
        ],
    )
    twist_marker = Node(
        package=PACKAGE_NAME,
        executable="twist_marker",
        name="twist_marker",
        namespace="/aiformula_visualization",
        output="screen",
        emulate_tty=True,
        parameters=[{"frame_id": "base_footprint", "scale": 1.0, "vertical_position": 2.0}],
        remappings=[
            ("twist", "/aiformula_control/twist_mux/cmd_vel"),
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

    return LaunchDescription(
        [
            *launch_args,
            twist_mux,
            twist_marker,
            rviz2,
            runtime_monitor,
        ]
    )
