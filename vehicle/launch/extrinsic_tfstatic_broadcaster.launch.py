import os.path as osp
from typing import Tuple
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from vehicle.vehicle_util import convert_xacro_to_urdf


def get_robot_state_publisher(
    context: LaunchContext,
    vehicle_name_lc: LaunchConfiguration,
    use_sim_time_lc: LaunchConfiguration
) -> Tuple[Node]:
    vehicle_name = context.perform_substitution(vehicle_name_lc)
    xacro_path = osp.join(get_package_share_directory("vehicle"),
                          "xacro", vehicle_name + ".xacro")
    urdf_path = osp.join(get_package_share_directory("vehicle"),
                         "xacro", vehicle_name + ".urdf")
    convert_xacro_to_urdf(xacro_path, urdf_path)
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    return (
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace="/aiformula_sensing",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time_lc,
            }],
            arguments=["--ros-args", "--log-level", "WARN"],  # No output
        ),
    )


def get_joint_state_publisher(
    context: LaunchContext,
    use_joint_state_publisher_lc: LaunchConfiguration,
    use_gui_lc: LaunchConfiguration,
    use_sim_time_lc: LaunchConfiguration
) -> Tuple[Node]:
    use_gui = context.perform_substitution(use_gui_lc).lower() == "true"
    node_name = "joint_state_publisher_gui" if use_gui else "joint_state_publisher"
    return (
        Node(
            package=node_name,
            executable=node_name,
            name=node_name,
            namespace="/aiformula_sensing",
            parameters=[{
                "use_sim_time": use_sim_time_lc,
            }],
            arguments=["--ros-args", "--log-level", "WARN"],  # No output
            condition=IfCondition(use_joint_state_publisher_lc),
        ),
    )


def generate_launch_description():
    launch_args = (
        DeclareLaunchArgument(
            "vehicle_name",
            default_value="ai_car1",
            description="Vehicle Name",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulation (Gazebo) clock",
        ),
        DeclareLaunchArgument(
            "use_joint_state_publisher",
            default_value="true",
            description="If true, use joint_state_publisher",
        ),
        DeclareLaunchArgument(
            "use_gui",
            default_value="false",
            description="If true, use joint_state_publisher_gui",
        ),
    )

    robot_state_publisher = OpaqueFunction(
        function=get_robot_state_publisher,
        args=[
            LaunchConfiguration("vehicle_name"),
            LaunchConfiguration("use_sim_time"),
        ],
    )
    joint_state_publisher = OpaqueFunction(
        function=get_joint_state_publisher,
        args=[
            LaunchConfiguration("use_joint_state_publisher"),
            LaunchConfiguration("use_gui"),
            LaunchConfiguration("use_sim_time"),
        ],
    )

    return LaunchDescription([
        *launch_args,
        robot_state_publisher,
        joint_state_publisher,
    ])
