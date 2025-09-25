import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from common_python.launch_util import check_zedx_available_fps


def get_zed_node(context):
    grab_resolution_val = LaunchConfiguration("grab_resolution").perform(context)
    grab_frame_rate_val = LaunchConfiguration("grab_frame_rate").perform(context)
    is_valid_fps = check_zedx_available_fps(grab_resolution_val, grab_frame_rate_val)
    
    # ComposableNode
    zed_wrapper_component = ComposableNode(
        package="zed_components",
        plugin="stereolabs::ZedCamera",
        name="zed_node",
        parameters=[
            LaunchConfiguration("config_common_path"),
            LaunchConfiguration("config_camera_path"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "general.grab_resolution": LaunchConfiguration("grab_resolution"),
                "general.grab_frame_rate": int(grab_frame_rate_val),
            },
        ],
        remappings=[
            ("~/left/image_rect_color", "/aiformula_sensing/zed_node/left_image/undistorted"),
            ("~/right/image_rect_color", "/aiformula_sensing/zed_node/right_image/undistorted"),
            ("~/imu/data", "/aiformula_sensing/zed_node/imu"),
        ],
    )

    return(
        # Component Container
        ComposableNodeContainer(
            name="zed_container",
            namespace="/aiformula_sensing",
            package="rclcpp_components",
            executable="component_container", 
            composable_node_descriptions=[zed_wrapper_component],
            output="screen",
            condition=IfCondition(str(is_valid_fps)),
        ),
    )


def generate_launch_description():
    launch_args = (
        DeclareLaunchArgument(
            "grab_resolution",
            default_value=TextSubstitution(text="HD1080"),
            description="The native camera grab resolution. HD1200, HD1080, SVGA",
            choices=["HD1200", "HD1080", "SVGA"],
        ),
        DeclareLaunchArgument(
            "grab_frame_rate",
            default_value=TextSubstitution(text="60"),
            description="grabbing rate (HD1200/HD1080: 60, 30, 15 - SVGA: 120, 60, 30, 15)",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Enable simulation time mode.", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "config_common_path",
            default_value=osp.join(get_package_share_directory("sample_vehicle"), "config", "zedx", "common.yaml"),
            description="Path to the common YAML configuration file.",
        ),
        DeclareLaunchArgument(
            "config_camera_path",
            default_value=osp.join(get_package_share_directory("sample_vehicle"), "config", "zedx", "zedx.yaml"),
            description="Path to the zedx YAML configuration file for the camera.",
        ),
    )
    zed_node = OpaqueFunction(function=get_zed_node)
    return LaunchDescription(
        [
            SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
            *launch_args,
            zed_node,
        ]
    )
