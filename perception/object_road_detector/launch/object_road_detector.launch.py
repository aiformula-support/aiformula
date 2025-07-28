import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    PACKAGE_NAME = "object_road_detector"
    ROS_PARAM_CONFIG = (osp.join(get_package_share_directory(PACKAGE_NAME), "config", "object_road_detector.yaml"),)

    launch_args = (
        DeclareLaunchArgument(
            "weight_path",
            default_value=osp.join(
                get_package_share_directory("object_road_detector"), "weights", "shiho-v1-20250321.pth"
            ),
            description="Path to the weight pth file.",
        ),
        DeclareLaunchArgument(
            "use_device",
            default_value="0",
            description="cuda device, i.e. 0 or cpu",
        ),
    )

    object_road_detector = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_perception",
        output="screen",
        parameters=[
            [*ROS_PARAM_CONFIG],
            # Overriding
            {
                "weight_path": LaunchConfiguration("weight_path"),
                "use_device": LaunchConfiguration("use_device"),
            },
        ],
        remappings=[
            ("sub_image", "/aiformula_sensing/zed_node/left_image/undistorted"),
            ("pub_mask_image", "/aiformula_perception/object_road_detector/mask_image"),
            ("pub_bbox", "/aiformula_perception/object_road_detector/rect"),
            ("pub_annotated_image", "/aiformula_visualization/object_road_detector/annotated_image"),
        ],
    )

    return LaunchDescription(
        [
            *launch_args,
            object_road_detector,
        ]
    )
