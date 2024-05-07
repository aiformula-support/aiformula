import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "odometry_publisher"
    NODE_NAME = "wheel_odometry_publisher"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)
    FRAME_IDS, TOPIC_NAMES = get_frame_ids_and_topic_names()

    launch_args = (
        DeclareLaunchArgument(
            "use_rosbag",
            default_value="false",
            description="If true, play rosbag",
        ),
        DeclareLaunchArgument(
            "rosbag_path",
            default_value="~/data/aiformula/20240328_shiho_can_imu_mag_gnss/test_01",
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

    ROS_PARAM_CONFIG = (
        osp.join(PACKAGE_DIR, "config", "wheel.yaml"),
    )
    wheel_odometry_publisher = Node(
        package=PACKAGE_NAME,
        executable=NODE_NAME,
        name=NODE_NAME,
        namespace="/aiformula_sensing",
        output="screen",
        emulate_tty=True,
        parameters=[*ROS_PARAM_CONFIG,
                    {
                        "odom_frame_id": FRAME_IDS["odom"],
                        "vehicle_frame_id": FRAME_IDS["base_footprint"],
                    }],
        remappings=[
            ("sub_can", TOPIC_NAMES["sensing"]["input_can_data"]),
            ("pub_odometry", TOPIC_NAMES["sensing"]["odometry"]["wheel"]),
        ],
    )
    rosbag_play = ExecuteProcess(
        cmd=[
            "ros2 bag play",
            " --topics ",
            TOPIC_NAMES["sensing"]["input_can_data"],
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

    return LaunchDescription([
        *launch_args,
        wheel_odometry_publisher,
        rosbag_play,
        rviz2,
    ])
