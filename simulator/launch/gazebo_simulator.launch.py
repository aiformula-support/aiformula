import os.path as osp
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    VEHICLE_NAME = "ai_car1"
    launch_args = (
        DeclareLaunchArgument(
            "world_name",
            default_value="shihou_course",
            description="World Name",
        ),
        DeclareLaunchArgument(
            "vehicle_name",
            default_value="ai_car1",
            description="Vehicle Name",
        ),
        DeclareLaunchArgument(
            "world_path",
            default_value=[
                osp.join(get_package_share_directory("simulator"), "worlds", ""),
                LaunchConfiguration("world_name"), "/",
                LaunchConfiguration("vehicle_name"), ".model"
            ],
            description="Path to world file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulation (Gazebo) clock",
        ),
    )

    tf_static_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("vehicle"),
                     "launch/extrinsic_tfstatic_broadcaster.launch.py"),
        ),
        launch_arguments={
            "vehicle_name": VEHICLE_NAME,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    # ExecuteProcess(
    # cmd=["export", "GAZEBO_MODEL_PATH=$(ros2 pkg prefix vehicle)/share/vehicle/xacro"],
    # shell=True),
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("gazebo_ros"),
                     "launch/gzserver.launch.py"),
        ),
        launch_arguments={
            "world": LaunchConfiguration("world_path"),
            "world_name": LaunchConfiguration("world_name"),
        }.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("gazebo_ros"),
                     "launch/gzclient.launch.py"),
        ),
    )
    set_use_sim_time = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/gazebo',
             'use_sim_time', LaunchConfiguration("use_sim_time")],
        output='screen'
    )

    return LaunchDescription([
        *launch_args,
        tf_static_publisher,
        gzserver,
        gzclient,
        set_use_sim_time,
    ])
