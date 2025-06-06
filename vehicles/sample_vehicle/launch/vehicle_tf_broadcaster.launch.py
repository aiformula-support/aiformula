import os.path as osp
from typing import Tuple
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from vehicle.vehicle_util import convert_xacro_to_urdf

PACKAGE_NAME = "sample_vehicle"
PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)


def get_robot_state_publisher(context: LaunchContext) -> Tuple[Node]:
    vehicle_name = LaunchConfiguration("vehicle_name").perform(context)
    xacro_path = osp.join(PACKAGE_DIR, "xacro", vehicle_name + ".xacro")
    urdf_path = osp.join(PACKAGE_DIR, "xacro", vehicle_name + ".urdf")
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
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            arguments=["--ros-args", "--log-level", "WARN"],  # No output
        ),
    )


def get_joint_state_publisher(context: LaunchContext) -> Tuple[Node]:
    use_gui = context.perform_substitution(LaunchConfiguration("use_gui")).lower() == "true"
    node_name = "joint_state_publisher_gui" if use_gui else "joint_state_publisher"
    return (
        Node(
            package=node_name,
            executable=node_name,
            name=node_name,
            namespace="/aiformula_sensing",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            arguments=["--ros-args", "--log-level", "WARN"],  # No output
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
            "use_gui",
            default_value="false",
            description="If true, use joint_state_publisher_gui",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="If true, run rviz2",
        ),
    )

    robot_state_publisher = OpaqueFunction(function=get_robot_state_publisher)

    joint_state_publisher = OpaqueFunction(function=get_joint_state_publisher)

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_robot_static_tf",
        arguments=["-d", osp.join(PACKAGE_DIR, "rviz", "vehicle.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        *launch_args,
        robot_state_publisher,
        joint_state_publisher,
        rviz2,
    ])
