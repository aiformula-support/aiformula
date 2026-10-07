import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    PACKAGE_NAME = "object_road_detector_trt"
    default_engine = osp.join(
        get_package_share_directory("object_road_detector"),
        "weights",
        "shiho-v1-20250321-1920x640.engine",
    )
    ROS_PARAM_CONFIG = (
        osp.join(get_package_share_directory(PACKAGE_NAME), "config", "object_road_detector_trt.yaml"),
    )

    launch_args = (
        DeclareLaunchArgument(
            "engine_path",
            default_value=default_engine,
            description="Path to the TensorRT engine produced by object_road_detector/tools/export_engine.py",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="If true, publish the annotated source image",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "enable_profiling",
            default_value="false",
            description="If true, log a periodic callback-rate summary",
            choices=["true", "false"],
        ),
    )

    detector = Node(
        package=PACKAGE_NAME,
        executable="object_road_detector_trt_node",
        name="object_road_detector",
        namespace="/aiformula_perception",
        output="screen",
        parameters=[
            [*ROS_PARAM_CONFIG],
            {
                "engine_path": LaunchConfiguration("engine_path"),
                "debug": LaunchConfiguration("debug"),
                "profiling.enable": LaunchConfiguration("enable_profiling"),
            },
        ],
        remappings=[
            ("sub_image", "/aiformula_sensing/zed_node/left_image/undistorted"),
            ("pub_mask_image", "/aiformula_perception/object_road_detector/mask_image"),
            ("pub_bbox", "/aiformula_perception/object_road_detector/rect"),
            ("pub_annotated_image", "/aiformula_visualization/object_road_detector/annotated_image"),
        ],
    )

    return LaunchDescription([*launch_args, detector])
