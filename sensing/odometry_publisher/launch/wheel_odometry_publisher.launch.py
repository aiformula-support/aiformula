import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    PACKAGE_NAME = "odometry_publisher"
    NODE_NAME = "wheel_odometry_publisher"
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
            default_value="/workspace/data/20240328_shiho_can_imu_mag_gnss/test_01",
            description="Path of rosbag to play",
        ),
        DeclareLaunchArgument(
            "rosbag_play_speed",
            default_value="3.0",
            description="Speed to play rosbag",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="If true, run rviz2",
        ),
    )

    ROS_PARAM_CONFIG = (osp.join(get_package_share_directory("sample_vehicle"), "config", "wheel.yaml"),)
    wheel_odometry_publisher = Node(
        package=PACKAGE_NAME,
        executable=NODE_NAME,
        name=NODE_NAME,
        namespace="/aiformula_sensing",
        output="screen",
        emulate_tty=True,
        # arguments=["--ros-args", "--log-level", ["aiformula_sensing.", NODE_NAME, ":=", LaunchConfiguration("logger")]],
        parameters=[
            *ROS_PARAM_CONFIG,
            {
                "odom_frame_id": "odom",
                "vehicle_frame_id": "base_footprint",
            },
        ],
        remappings=[
            ("sub_can", "/aiformula_sensing/vehicle_info"),
            ("pub_odometry", "/aiformula_sensing/wheel_odometry_publisher/odom"),
        ],
    )
    rosbag_play = ExecuteProcess(
        cmd=[
            "ros2 bag play",
            " --topics ",
            "/aiformula_sensing/vehicle_info",
            "/tf",
            "/tf_static",
            " -r ",
            LaunchConfiguration("rosbag_play_speed"),
            " -- ",
            LaunchConfiguration("rosbag_path"),
        ],
        condition=IfCondition(LaunchConfiguration("use_rosbag")),
        shell=True,
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_wheel_odometry_publisher",
        arguments=["-d", osp.join(PACKAGE_DIR, "rviz", NODE_NAME + ".rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            *launch_args,
            wheel_odometry_publisher,
            rosbag_play,
            rviz2,
        ]
    )
