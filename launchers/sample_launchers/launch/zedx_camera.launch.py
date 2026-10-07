import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

from common_python.launch_util import check_zedx_available_fps


def launch_setup(context, *args, **kwargs):
    return_array = []
    grab_resolution_val = LaunchConfiguration("grab_resolution").perform(context)
    grab_frame_rate_val = LaunchConfiguration("grab_frame_rate").perform(context)
    is_valid_fps = check_zedx_available_fps(grab_resolution_val, grab_frame_rate_val)

    zed_container = ComposableNodeContainer(
        name="zed_container",
        namespace="aiformula_sensing",
        package="rclcpp_components",
        executable="component_container_isolated",
        arguments=["--use_multi_threaded_executor", "--ros-args", "--log-level", "info"],
        condition=IfCondition(str(is_valid_fps)),
        output="screen",
        composable_node_descriptions=[],
    )
    return_array.append(zed_container)

    # ZED Wrapper component
    zed_wrapper_component = ComposableNode(
        package="zed_components",
        namespace="aiformula_sensing",
        plugin="stereolabs::ZedCamera",
        name="zed_node",
        parameters=[
            # YAML files
            LaunchConfiguration("config_common_path"),
            LaunchConfiguration("config_camera_path"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "general.camera_name": LaunchConfiguration("camera_name").perform(context),
                "general.grab_resolution": LaunchConfiguration("grab_resolution"),
                "general.grab_frame_rate": int(grab_frame_rate_val),
                "general.pub_resolution": LaunchConfiguration("pub_resolution"),
                "general.pub_downscale_factor": float(
                    LaunchConfiguration("pub_downscale_factor").perform(context)
                ),
                "general.pub_frame_rate": float(
                    LaunchConfiguration("pub_frame_rate").perform(context)
                ),
                "depth.depth_mode": LaunchConfiguration("depth_mode").perform(context),
                "depth.depth_stabilization": LaunchConfiguration("depth_stabilization"),
                "depth.point_cloud_res": LaunchConfiguration("point_cloud_res").perform(context),
                "depth.point_cloud_freq": float(
                    LaunchConfiguration("point_cloud_freq").perform(context)
                ),
                "pos_tracking.publish_tf": LaunchConfiguration("publish_tf"),
            },
        ],
        remappings=[
            ("~/left/image_rect_color", "/aiformula_sensing/zed_node/left_image/undistorted"),
            ("~/right/image_rect_color", "/aiformula_sensing/zed_node/right_image/undistorted"),
            ("~/imu/data", "/aiformula_sensing/zed_node/imu"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    full_container_name = "/" + "aiformula_sensing" + "/" + "zed_container"
    load_zed = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[zed_wrapper_component],
    )
    detector_component = ComposableNode(
        package="object_road_detector_trt",
        plugin="aiformula::ObjectRoadDetectorTrt",
        name="object_road_detector",
        namespace="aiformula_perception",
        parameters=[
            osp.join(
                get_package_share_directory("object_road_detector_trt"),
                "config",
                "object_road_detector_trt.yaml",
            ),
            {
                "engine_path": LaunchConfiguration("engine_path").perform(context),
                "debug": LaunchConfiguration("detector_debug").perform(context) == "true",
                "profiling.enable": LaunchConfiguration("detector_profiling").perform(context) == "true",
            },
        ],
        remappings=[
            ("sub_image", "/aiformula_sensing/zed_node/left_image/undistorted"),
            ("pub_mask_image", "/aiformula_perception/object_road_detector/mask_image"),
            ("pub_bbox", "/aiformula_perception/object_road_detector/rect"),
            ("pub_annotated_image", "/aiformula_visualization/object_road_detector/annotated_image"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    load_detector = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[detector_component],
        condition=IfCondition(LaunchConfiguration("enable_object_road_detector")),
    )
    return_array.extend([load_zed, load_detector])
    return return_array


def generate_launch_description():
    launch_args = (
        DeclareLaunchArgument(
            "camera_name",
            default_value=TextSubstitution(text="zed"),
            description="The name of the camera. It can be different from the camera model",
        ),
        DeclareLaunchArgument(
            "grab_resolution",
            default_value=TextSubstitution(text="HD1080"),
            description="The native camera grab resolution. HD1200, HD1080, SVGA",
            choices=["HD1200", "HD1080", "SVGA"],
        ),
        DeclareLaunchArgument(
            "grab_frame_rate",
            default_value=TextSubstitution(text="30"),
            description="grabbing rate (HD1200/HD1080: 60, 30, 15 - SVGA: 120, 60, 30, 15)",
        ),
        DeclareLaunchArgument(
            "pub_resolution",
            default_value=TextSubstitution(text="CUSTOM"),
            description="The resolution used for output. 'NATIVE' to use the same `general.grab_resolution` - `CUSTOM` to apply the `general.pub_downscale_factor` downscale factory to reduce bandwidth in transmission",
        ),
        DeclareLaunchArgument(
            "pub_downscale_factor",
            default_value=TextSubstitution(text="1.5"),
            description="rescale factor used to rescale image before publishing when 'pub_resolution' is 'CUSTOM'. 1.5 turns the HD1080 grab into a 1280x720 publish; use 'pub_resolution:=NATIVE' for full 1920x1080",
        ),
        DeclareLaunchArgument(
            "pub_frame_rate",
            default_value=TextSubstitution(text="15.0"),
            description="publish frame rate frequency of publishing of visual images and depth images. This is the hard ceiling on the perception output rate",
        ),
        DeclareLaunchArgument(
            "depth_mode",
            default_value="NEURAL_LIGHT",
            description="Matches the ZED SDK setting: 'NONE', 'PERFORMANCE', 'QUALITY', 'ULTRA', 'NEURAL', 'NEURAL_LIGHT', 'NEURAL_PLUS'",
        ),
         DeclareLaunchArgument(
            "depth_stabilization",
            default_value=TextSubstitution(text="0"),
            description="# Forces positional tracking to start if major than 0 - Range: [0,100]",
        ),
        DeclareLaunchArgument(
            "point_cloud_res",
            default_value="REDUCED",
            description="Point cloud publish resolution. 'COMPACT' is the grab resolution, 'REDUCED' is half of it",
            choices=["COMPACT", "REDUCED", "PUB", "FULL"],
        ),
        DeclareLaunchArgument(
            "point_cloud_freq",
            default_value=TextSubstitution(text="5.0"),
            description="Point cloud publish rate [Hz]. Independent of 'pub_frame_rate' and must not exceed 'grab_frame_rate'",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="false",
            description="Enable publication of the `odom -> camera_link` TF.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Enable simulation time mode.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "config_common_path",
            default_value=osp.join(
                get_package_share_directory("zed_wrapper"), "config", "common_stereo.yaml"
            ),
            description="Path to the common YAML configuration file.",
        ),
        DeclareLaunchArgument(
            "config_camera_path",
            default_value=osp.join(
                get_package_share_directory("zed_wrapper"), "config", "zedx.yaml"
            ),
            description="Path to the zedx YAML configuration file for the camera.",
        ),
        DeclareLaunchArgument(
            "enable_object_road_detector",
            default_value="true",
            description="Load the TensorRT detector into the same container as the ZED wrapper",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "engine_path",
            default_value=osp.join(
                get_package_share_directory("object_road_detector"),
                "weights",
                "shiho-v1-20250321-1920x640.engine",
            ),
            description="Path to the TensorRT engine produced by object_road_detector/tools/export_engine.py",
        ),
        DeclareLaunchArgument(
            "detector_debug",
            default_value="false",
            description="If true, the detector publishes an annotated image",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "detector_profiling",
            default_value="false",
            description="If true, the detector logs a periodic callback-rate summary",
            choices=["true", "false"],
        ),
    )
    zed_node = OpaqueFunction(function=launch_setup)
    return LaunchDescription(
        [
            *launch_args,
            zed_node,
        ]
    )
