import os.path as osp
from typing import Tuple

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sample_vehicle.vehicle_util import get_zed_intrinsic_param_path

PACKAGE_NAME = "object_publisher"
PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)


def create_object_publisher_node(context: LaunchContext) -> Tuple[Node]:
    camera_sn = LaunchConfiguration("camera_sn").perform(context)
    camera_resolution = LaunchConfiguration("camera_resolution").perform(context)

    ROS_PARAM_CONFIG = (
        osp.join(PACKAGE_DIR, "config", "object_publisher.yaml"),
        osp.join(PACKAGE_DIR, "config", "tracked_object.yaml"),
        get_zed_intrinsic_param_path(camera_sn, camera_resolution),
    )
    return (
        Node(
            package=PACKAGE_NAME,
            executable=PACKAGE_NAME,
            name=PACKAGE_NAME,
            namespace="/aiformula_perception",
            output="screen",
            emulate_tty=True,
            # arguments=[
            #     "--ros-args",
            #     "--log-level",
            #     ["aiformula_perception.", PACKAGE_NAME, ":=", LaunchConfiguration("logger")],
            # ],
            parameters=[
                *ROS_PARAM_CONFIG,
                {
                    "camera_name": LaunchConfiguration("camera_name"),
                    "camera_frame_id": "zed_left_camera_optical_frame",
                    "vehicle_frame_id": "base_footprint",
                    "odom_frame_id": "odom",
                    "debug": LaunchConfiguration("debug"),
                },
            ],
            remappings=[
                ("sub_bbox", "/aiformula_perception/object_road_detector/rect"),
                ("pub_object", "/aiformula_perception/object_publisher/object_info"),
                ("pub_unfiltered_object", "/aiformula_visualization/object_publisher/unfiltered_object"),
            ],
        ),
    )


def generate_launch_description():
    launch_args = (
        DeclareLaunchArgument(
            "logger",
            default_value="info",
            choices=["debug", "info", "warn", "error", "fatal"],
            description="Ros logger level",
        ),
        DeclareLaunchArgument(
            "camera_name",
            default_value="zedx",
            description="Camera name passed to getCameraParams() to load the corresponding configuration",
        ),
        DeclareLaunchArgument(
            "camera_sn",
            default_value="SN48311510",
            description="Camera serial number",
        ),
        DeclareLaunchArgument(
            "camera_resolution",
            default_value="nHD",
            description="Camera image resolution",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="If true, run rviz2",
        ),
        DeclareLaunchArgument(
            "use_rosbag",
            default_value="false",
            description="If true, play rosbag",
        ),
        DeclareLaunchArgument(
            "rosbag_path",
            default_value="~/data/aiformula/20250219_bbox_bboximage_odom_tf/2_rect/2_rect_vehicle_info",
            description="Path of rosbag to play",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="If true, publish unfiltered object pose",
        ),
    )

    object_publisher = OpaqueFunction(function=create_object_publisher_node)

    topics = [
        "/aiformula_perception/object_road_detector/rect",
        "/aiformula_visualization/object_road_detector/annotated_image",
        "/tf",
        "/tf_static",
    ]
    rosbag_play = ExecuteProcess(
        cmd=[
            "ros2 bag play",
            " --topics ",
            " ".join(topics),
            " -- ",
            LaunchConfiguration("rosbag_path"),
        ],
        condition=IfCondition(LaunchConfiguration("use_rosbag")),
        shell=True,
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_object_publisher",
        arguments=["-d", osp.join(PACKAGE_DIR, "rviz", PACKAGE_NAME + ".rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    return LaunchDescription(
        [
            *launch_args,
            object_publisher,
            rosbag_play,
            rviz2,
        ]
    )
