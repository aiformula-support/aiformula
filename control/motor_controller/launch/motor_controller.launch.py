import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    PACKAGE_NAME = "motor_controller"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)

    launch_args = (
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logger level (debug, info, warn, error, fatal)",
        ),
    )

    ROS_PARAM_CONFIG = (
        osp.join(get_package_share_directory("sample_vehicle"), "config", "wheel.yaml"),
        osp.join(PACKAGE_DIR, "config", "motor_controller.yaml"),
    )
    motor_controller = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_control",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration('log_level')],
        parameters=[*ROS_PARAM_CONFIG],
        remappings=[
            ("sub_speed_command", "/aiformula_control/twist_mux/cmd_vel"),
            ("pub_can", "/aiformula_control/motor_controller/reference_signal"),
        ],
    )

    return LaunchDescription([
        *launch_args,
        motor_controller,
    ])
