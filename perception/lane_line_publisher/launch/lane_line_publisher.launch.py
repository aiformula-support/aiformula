import os.path as osp
from typing import Tuple

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, evaluate_condition_expression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sample_vehicle.vehicle_util import get_zed_intrinsic_param_path


def create_lane_line_publisher_node(context: LaunchContext) -> Tuple[Node]:
    package_dir = get_package_share_directory("lane_line_publisher")
    config_path = osp.join(package_dir, "config", "lane_line_publisher.yaml")
    camera_sn = LaunchConfiguration("camera_sn").perform(context)
    camera_resolution = LaunchConfiguration("camera_resolution").perform(context)
    camera_params_path = get_zed_intrinsic_param_path(camera_sn, camera_resolution)
    debug = evaluate_condition_expression(context, [LaunchConfiguration("debug")])

    return (
        Node(
            package="lane_line_publisher",
            executable="lane_line_publisher",
            name="lane_line_publisher",
            namespace="/aiformula_perception",
            output="screen",
            emulate_tty=True,
            parameters=[
                config_path,
                camera_params_path,
                {
                    "vehicle_frame_id": "base_footprint",
                    "camera_frame_id": "zed_left_camera_optical_frame",
                    "camera_name": "zedx",
                    "debug": debug,
                },
            ],
            remappings=[
                ("mask_image", "/aiformula_perception/object_road_detector/mask_image"),
                ("annotated_mask_image", "/aiformula_perception/lane_line_publisher/annotated_mask_image"),
                ("lane_lines/left", "/aiformula_perception/lane_line_publisher/lane_lines/left"),
                ("lane_lines/right", "/aiformula_perception/lane_line_publisher/lane_lines/right"),
                ("lane_lines/center", "/aiformula_perception/lane_line_publisher/lane_lines/center"),
            ],
        ),
    )


def create_rviz_node(context: LaunchContext) -> Tuple[Node]:
    package_dir = get_package_share_directory("lane_line_publisher")
    rviz_path = osp.join(package_dir, "rviz", "lane_line_publisher.rviz")
    return (
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_lane_line_publisher",
            arguments=["-d", rviz_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    )


def generate_launch_description():
    launch_args = (
        DeclareLaunchArgument(
            "camera_name",
            default_value="zedx",
            description="camera name",
        ),
        DeclareLaunchArgument(
            "camera_sn",
            default_value="SN48311510",
            description="camera serial number",
        ),
        DeclareLaunchArgument(
            "camera_resolution",
            default_value="nHD",
            description="camera image resolution",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="enable debug publications",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="launch rviz",
        ),
    )

    return LaunchDescription(
        [
            *launch_args,
            OpaqueFunction(function=create_lane_line_publisher_node),
            OpaqueFunction(function=create_rviz_node),
        ]
    )
