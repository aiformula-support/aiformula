import os.path as osp

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import (
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode
from common_python.launch_util import check_zedx_available_fps

def launch_setup(context, *args, **kwargs):
    return_array = []

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_tf = LaunchConfiguration('publish_tf')
    container_name_val = LaunchConfiguration('container_name').perform(context)
    namespace_val = LaunchConfiguration('namespace').perform(context)
    camera_name_val = LaunchConfiguration('camera_name').perform(context)
    grab_resolution_val = LaunchConfiguration("grab_resolution").perform(context)
    grab_frame_rate_val = LaunchConfiguration("grab_frame_rate").perform(context)
    pub_downscale_factor_val = LaunchConfiguration("pub_downscale_factor").perform(context)
    pub_frame_rate_val = LaunchConfiguration("pub_frame_rate").perform(context)
    depth_mode_val = LaunchConfiguration("depth_mode").perform(context)
    is_valid_fps = check_zedx_available_fps(grab_resolution_val, grab_frame_rate_val)
    container_exec='component_container_isolated'

    zed_container = ComposableNodeContainer(
            name=container_name_val,
            namespace=namespace_val,
            package='rclcpp_components',
            executable=container_exec,
            arguments=['--use_multi_threaded_executor','--ros-args', '--log-level', 'info'],
            condition=IfCondition(str(is_valid_fps)),
            output='screen',
            composable_node_descriptions=[]
    )
    return_array.append(zed_container)

    # ZED Node parameters
    node_parameters = [
            # YAML files
            LaunchConfiguration("config_common_path"),
            LaunchConfiguration("config_camera_path"),
    ]
    node_parameters.append( 
            # Launch arguments must override the YAML files values
            {
                'use_sim_time': use_sim_time,
                'general.camera_name': camera_name_val,
                "general.grab_resolution": LaunchConfiguration("grab_resolution"),
                "general.grab_frame_rate": int(grab_frame_rate_val),
                "general.pub_resolution": LaunchConfiguration("pub_resolution"),
                "general.pub_downscale_factor": float(pub_downscale_factor_val),
                "general.pub_frame_rate": float(pub_frame_rate_val),
                "depth.depth_mode": depth_mode_val,
                'pos_tracking.publish_tf': publish_tf,
            }
    )

    # ZED Wrapper component
    zed_wrapper_component = ComposableNode(
        package='zed_components',
        namespace=namespace_val,
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=node_parameters,
        remappings = [
            ("~/left/image_rect_color", "/aiformula_sensing/zed_node/left_image/undistorted"),
            ("~/right/image_rect_color", "/aiformula_sensing/zed_node/right_image/undistorted"),
            ("~/imu/data", "/aiformula_sensing/zed_node/imu"),
            ],
    )
    full_container_name = '/' + namespace_val + '/' + container_name_val
    load_composable_node = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[zed_wrapper_component]
    )
    return_array.append(load_composable_node)
    return return_array

def generate_launch_description():
    launch_args = (
        DeclareLaunchArgument(
            'camera_name',
            default_value=TextSubstitution(text='zed'),
            description='The name of the camera. It can be different from the camera model'
        ),
        DeclareLaunchArgument(
            'container_name',
            default_value='zed_container',
            description='The name of the container to be used to load the ZED component. If empty (default) a new container will be created.'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='aiformula_sensing',
            description='The namespace of the node. If empty (default) the camera name is used.'
        ),
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
            "pub_resolution",
            default_value=TextSubstitution(text="CUSTOM"),
            description="The resolution used for output. 'NATIVE' to use the same `general.grab_resolution` - `CUSTOM` to apply the `general.pub_downscale_factor` downscale factory to reduce bandwidth in transmission",
        ),
        DeclareLaunchArgument(
            "pub_downscale_factor",
            default_value=TextSubstitution(text="3.0"),
            description="rescale factor used to rescale image before publishing when 'pub_resolution' is 'CUSTOM'",
        ),
        DeclareLaunchArgument(
            "pub_frame_rate",
            default_value=TextSubstitution(text="15.0"),
            description="publish frame rate frequency of publishing of visual images and depth images",
        ),
        DeclareLaunchArgument(
            'depth_mode',
            default_value='NONE',
            description= "Matches the ZED SDK setting: 'NONE', 'PERFORMANCE', 'QUALITY', 'ULTRA', 'NEURAL', 'NEURAL_LIGHT', 'NEURAL_PLUS'",
        ),        
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='Enable publication of the `odom -> camera_link` TF.',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", 
            description="Enable simulation time mode.", 
            choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "config_common_path",
            default_value=osp.join(get_package_share_directory("zed_wrapper"), "config", "common_stereo.yaml"),
            description="Path to the common YAML configuration file.",
        ),
        DeclareLaunchArgument(
            "config_camera_path",
            default_value=osp.join(get_package_share_directory("zed_wrapper"), "config", "zedx.yaml"),
            description="Path to the zedx YAML configuration file for the camera.",
        ),
    )
    zed_node = OpaqueFunction(function=launch_setup)
    return LaunchDescription(
        [
            *launch_args,
            zed_node,
        ]
    )
