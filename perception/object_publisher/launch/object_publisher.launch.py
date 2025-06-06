import os.path as osp
from typing import Tuple

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

from common_python.launch_util import get_frame_ids_and_topic_names
from vehicle.vehicle_util import get_zed_intrinsic_param_path


PACKAGE_NAME = "object_publisher"
PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)
FRAME_IDS, TOPIC_NAMES = get_frame_ids_and_topic_names()


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
            parameters=[*ROS_PARAM_CONFIG,
                        {
                            "camera_name": LaunchConfiguration("camera_name"),
                            "camera_frame_id": FRAME_IDS["zedx"]["left"],
                            "vehicle_frame_id": FRAME_IDS["base_footprint"],
                            "odom_frame_id": FRAME_IDS["odom"],
                            "debug": LaunchConfiguration("debug"),
                        }],
            remappings=[
                ("sub_bbox", TOPIC_NAMES["perception"]["objects"]["bounding_box"]),
                ("pub_object", TOPIC_NAMES["perception"]["objects"]["info"]),
                ("pub_unfiltered_object", TOPIC_NAMES["visualization"]["unfiltered_object"]),
            ],
        ),
    )


def generate_launch_description():
    launch_args = (
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
        TOPIC_NAMES["perception"]["objects"]["bounding_box"],
        TOPIC_NAMES["visualization"]["annotated_image"],
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
    return LaunchDescription([
        *launch_args,
        object_publisher,
        rosbag_play,
        rviz2,
    ])
